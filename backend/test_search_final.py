#!/usr/bin/env python3
"""
Test script to verify Qdrant search functionality with correct method
"""
import os
from openai import OpenAI
from qdrant_client import QdrantClient
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Initialize clients
openai_client = OpenAI(
    api_key=os.getenv("OPENROUTER_API_KEY"),
    base_url="https://openrouter.ai/api/v1"
)

qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)

# Test embedding and search
try:
    print("Testing embedding and search with correct method...")

    # Generate embedding for test query
    response = openai_client.embeddings.create(
        model="BAAI/bge-large-en-v1.5",
        input=["What is ROS 2?"]
    )

    query_embedding = response.data[0].embedding
    print(f"Query embedding generated with dimension: {len(query_embedding)}")

    # Perform search in Qdrant using the correct method
    results = qdrant_client.search(
        collection_name="rag_embedding",
        query_vector=query_embedding,
        limit=3
    )

    print(f"Found {len(results)} results:")
    for i, result in enumerate(results):
        print(f"Result {i+1}:")
        print(f"  Score: {result.score}")
        print(f"  Text preview: {result.payload.get('text', '')[:100]}...")
        print(f"  URL: {result.payload.get('url', '')}")
        print()

    # Test if results are empty
    if len(results) == 0:
        print("No results found. Testing if the collection has data...")
        count = qdrant_client.count(collection_name="rag_embedding")
        print(f"Total points in collection: {count}")

        # Get a few random points to verify data exists
        sample_points = qdrant_client.scroll(
            collection_name="rag_embedding",
            limit=2
        )
        print(f"Sample points retrieved: {len(sample_points[0])}")
        for i, point in enumerate(sample_points[0]):
            print(f"Sample {i+1}: {point.payload.get('text', '')[:100]}...")

except Exception as e:
    print(f"Error in search test: {e}")
    import traceback
    traceback.print_exc()