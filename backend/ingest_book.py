#!/usr/bin/env python3
"""
Book Ingestion Script for RAG System

This script re-ingests the entire Physical AI & Humanoid Robotics book
into Qdrant Cloud from scratch using the Qwen/qwen3-embedding-0.6B model via OpenRouter.
"""

import os
import requests
import time
from typing import List, Dict, Any, Optional
from urllib.parse import urljoin, urlparse
import re
from bs4 import BeautifulSoup
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.models import PointStruct
from openai import OpenAI
from dotenv import load_dotenv
import logging

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class BookIngestionPipeline:
    def __init__(self):
        """Initialize the ingestion pipeline with configuration."""
        self.openrouter_api_key = os.getenv("OPENROUTER_API_KEY")  # Use API key from environment
        self.qdrant_url = os.getenv("QDRANT_URL")
        self.qdrant_api_key = os.getenv("QDRANT_API_KEY")
        self.collection_name = os.getenv("COLLECTION_NAME", "rag_embedding")
        self.embedding_model = "BAAI/bge-large-en-v1.5"  # Use a free and effective embedding model available on OpenRouter

        # Initialize clients
        self.openai_client = OpenAI(
            api_key=self.openrouter_api_key,
            base_url="https://openrouter.ai/api/v1"
        )
        self.qdrant_client = QdrantClient(
            url=self.qdrant_url,
            api_key=self.qdrant_api_key,
            timeout=30
        )

        # Base URL for the book website
        self.base_url = "https://physicalairobotics.netlify.app"

    def get_all_urls(self) -> List[str]:
        """
        Crawl the target website and return all discoverable URLs.
        """
        logger.info(f"Starting to crawl website: {self.base_url}")
        urls = set()
        visited = set()
        to_visit = [self.base_url]

        # Limit the crawl to avoid infinite loops
        max_pages = 100
        pages_crawled = 0

        while to_visit and pages_crawled < max_pages:
            current_url = to_visit.pop(0)

            if current_url in visited:
                continue

            visited.add(current_url)
            pages_crawled += 1

            try:
                response = requests.get(current_url, timeout=10)
                if response.status_code == 200:
                    soup = BeautifulSoup(response.content, 'html.parser')

                    # Extract all links
                    for link in soup.find_all('a', href=True):
                        href = link['href']
                        full_url = urljoin(current_url, href)

                        # Only include URLs from the same domain and with meaningful paths
                        if self.base_url in full_url and full_url not in visited:
                            if not any(ext in full_url for ext in ['.pdf', '.jpg', '.png', '.css', '.js']):
                                urls.add(full_url)
                                to_visit.append(full_url)

                logger.info(f"Crawled {current_url}, found {len(urls)} URLs so far")

                # Rate limiting
                time.sleep(0.1)

            except Exception as e:
                logger.error(f"Error crawling {current_url}: {str(e)}")
                continue

        logger.info(f"Completed crawl, found {len(urls)} URLs")
        return list(urls)

    def extract_text_from_url(self, url: str) -> Optional[Dict[str, Any]]:
        """
        Extract clean text content from a given URL.
        """
        try:
            response = requests.get(url, timeout=10)
            if response.status_code != 200:
                logger.error(f"Failed to fetch {url}, status: {response.status_code}")
                return None

            soup = BeautifulSoup(response.content, 'html.parser')

            # Remove script and style elements
            for script in soup(["script", "style"]):
                script.decompose()

            # Try to find main content area (common selectors for Docusaurus sites)
            content_selectors = [
                'main article',  # Docusaurus main content
                'main',  # Alternative main content
                '.markdown',  # Markdown content
                '.container',  # Container with content
                'article',  # Article content
                '.content',  # Generic content class
                'body'  # Fallback
            ]

            text_content = ""
            section_title = ""

            for selector in content_selectors:
                elements = soup.select(selector)
                if elements:
                    for element in elements:
                        # Get text but try to preserve some structure
                        text_content += element.get_text(separator='\\n', strip=True) + "\\n\\n"
                    break

            # Get the page title
            title_elem = soup.find('title')
            section_title = title_elem.get_text().strip() if title_elem else urlparse(url).path.split('/')[-1]

            # Clean up the text
            text_content = re.sub(r'\\n+', '\\n', text_content).strip()

            if not text_content.strip():
                logger.warning(f"No content extracted from {url}")
                return None

            return {
                'url': url,
                'section_title': section_title,
                'text': text_content
            }

        except Exception as e:
            logger.error(f"Error extracting text from {url}: {str(e)}")
            return None

    def chunk_text(self, text: str, max_chunk_size: int = 800, overlap: int = 100) -> List[str]:
        """
        Split text into appropriately sized chunks for embedding.
        """
        sentences = re.split(r'[.!?]+', text)
        chunks = []
        current_chunk = ""

        for sentence in sentences:
            sentence = sentence.strip()
            if not sentence:
                continue

            # Check if adding this sentence would exceed the limit
            if len(current_chunk) + len(sentence) > max_chunk_size and current_chunk:
                chunks.append(current_chunk.strip())

                # Add overlap by including some of the previous content
                if overlap > 0:
                    # Take the last 'overlap' characters from the current chunk to start the next
                    overlap_text = current_chunk[-overlap:] if len(current_chunk) > overlap else current_chunk
                    current_chunk = overlap_text + " " + sentence
                else:
                    current_chunk = sentence
            else:
                current_chunk += " " + sentence if current_chunk else sentence

        # Add the last chunk if it has content
        if current_chunk.strip():
            chunks.append(current_chunk.strip())

        # Filter out very small chunks
        chunks = [chunk for chunk in chunks if len(chunk) > 50]

        return chunks

    def embed(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for text chunks using OpenRouter Qwen API.
        """
        try:
            response = self.openai_client.embeddings.create(
                model=self.embedding_model,
                input=texts
            )
            return [item.embedding for item in response.data]
        except Exception as e:
            logger.error(f"Error generating embeddings: {str(e)}")
            raise

    def create_collection(self):
        """
        Create a Qdrant collection for storing embeddings.
        """
        logger.info(f"Creating collection '{self.collection_name}' with Qwen embedding model")

        # Delete collection if it exists (to start fresh)
        try:
            self.qdrant_client.delete_collection(self.collection_name)
            logger.info(f"Deleted existing collection '{self.collection_name}'")
        except Exception as e:
            logger.info(f"Collection '{self.collection_name}' did not exist: {str(e)}")

        # Create new collection with appropriate vector size for Qwen model
        # Qwen3-embedding-0.6b produces 1024-dimensional vectors
        self.qdrant_client.create_collection(
            collection_name=self.collection_name,
            vectors_config=models.VectorParams(
                size=1024,  # Qwen3-embedding-0.6b produces 1024-dimensional vectors
                distance=models.Distance.COSINE
            )
        )

        logger.info(f"Created collection '{self.collection_name}' with 1024-dimensional vectors")

    def save_chunk_to_qdrant(self, chunk: str, embedding: List[float], metadata: Dict[str, Any], chunk_index: int):
        """
        Save a single chunk with its embedding to Qdrant.
        """
        try:
            # Create a valid point ID using UUID instead of hash
            import uuid
            point_id = str(uuid.uuid4())

            point = PointStruct(
                id=point_id,  # Use a valid ID
                vector=embedding,
                payload={
                    'text': chunk,
                    'url': metadata['url'],
                    'section_title': metadata['section_title'],
                    'chunk_index': chunk_index
                }
            )

            self.qdrant_client.upsert(
                collection_name=self.collection_name,
                points=[point]
            )

            return True
        except Exception as e:
            logger.error(f"Error saving chunk to Qdrant: {str(e)}")
            return False

    def embed_with_retry(self, texts: List[str], max_retries: int = 5) -> List[List[float]]:
        """
        Generate embeddings with retry logic and rate limiting handling.
        Be conservative with API calls to respect limits.
        """
        for attempt in range(max_retries):
            try:
                # Respect rate limits - sleep before each call to be safe
                time.sleep(1)  # Wait 1 second before each embedding call to stay within limits

                response = self.openai_client.embeddings.create(
                    model=self.embedding_model,
                    input=texts
                )
                return [item.embedding for item in response.data]
            except Exception as e:
                logger.error(f"OpenRouter API error on attempt {attempt+1}: {str(e)}")
                if "rate limit" in str(e).lower() or "429" in str(e):
                    logger.warning(f"Rate limit hit on attempt {attempt+1}, waiting 60 seconds before retry...")
                    time.sleep(60)  # Wait 60 seconds before retrying due to rate limit
                elif attempt == max_retries - 1:  # Last attempt
                    raise
                else:
                    time.sleep(10)  # Wait 10 seconds before retrying other errors

        raise Exception(f"Failed to generate embeddings after {max_retries} attempts")

    def run_ingestion(self):
        """
        Execute the full ingestion pipeline.
        """
        logger.info("Starting book ingestion pipeline...")

        # 1. Create collection (this will delete any existing collection)
        self.create_collection()

        # 2. Get all URLs from the website
        urls = self.get_all_urls()
        logger.info(f"Found {len(urls)} URLs to process")

        # 3. Process each URL
        total_chunks = 0
        for i, url in enumerate(urls):
            logger.info(f"Processing URL {i+1}/{len(urls)}: {url}")

            # Extract content from URL
            content = self.extract_text_from_url(url)
            if not content:
                logger.warning(f"Could not extract content from {url}")
                continue

            # Chunk the text
            chunks = self.chunk_text(content['text'])
            logger.info(f"Created {len(chunks)} chunks from {url}")

            # Process each chunk
            for j, chunk in enumerate(chunks):
                try:
                    # Generate embedding for the chunk
                    embeddings = self.embed_with_retry([chunk])
                    embedding = embeddings[0]

                    # Save to Qdrant
                    success = self.save_chunk_to_qdrant(chunk, embedding, content, j)
                    if success:
                        total_chunks += 1
                        logger.info(f"Saved chunk {j+1} from {url} to Qdrant")

                    # Conservative rate limiting to avoid overwhelming the API
                    time.sleep(3)  # Wait 3 seconds between chunks to stay within API limits (reduced from 15)

                except Exception as e:
                    logger.error(f"Error processing chunk {j+1} from {url}: {str(e)}")
                    continue

        logger.info(f"Ingestion completed! Total chunks saved: {total_chunks}")
        return total_chunks


def main():
    """Main function to run the ingestion pipeline."""
    logger.info("Initializing book ingestion pipeline with Qwen embeddings")

    pipeline = BookIngestionPipeline()

    # Verify configuration
    if not all([pipeline.openrouter_api_key, pipeline.qdrant_url, pipeline.qdrant_api_key]):
        logger.error("Missing required environment variables. Please check your .env file.")
        return

    logger.info("Starting ingestion process with Qwen embeddings...")
    total_chunks = pipeline.run_ingestion()

    logger.info(f"Ingestion completed successfully! {total_chunks} chunks ingested.")


if __name__ == "__main__":
    main()