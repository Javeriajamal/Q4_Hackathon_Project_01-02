#!/usr/bin/env python3
"""
Test script to verify the full pipeline - embedding, search, and chat
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

# Test the full pipeline
try:
    print("Testing full pipeline...")

    # Step 1: Generate embedding for query
    print("Step 1: Generating embedding...")
    embed_response = openai_client.embeddings.create(
        model="BAAI/bge-large-en-v1.5",
        input=["What is ROS 2?"]
    )
    query_embedding = embed_response.data[0].embedding
    print(f"V Embedding generated with dimension: {len(query_embedding)}")

    # Step 2: Search in Qdrant
    print("Step 2: Searching in Qdrant...")
    search_results = qdrant_client.query_points(
        collection_name="rag_embedding",
        query=query_embedding,
        limit=3
    )

    contexts = [r.payload.get("text", "") for r in search_results.points]
    context = "\n".join(contexts) or "No relevant context found."

    print(f"V Found {len(contexts)} context chunks")
    print(f"Context preview: {context[:200].encode('ascii', errors='ignore').decode('ascii')}...")

    # Step 3: Chat with OpenRouter
    print("Step 3: Chatting with OpenRouter...")
    chat_response = openai_client.chat.completions.create(
        model="Qwen/Qwen3-72B-Instruct",  # Updated model name
        messages=[
            {
                "role": "system",
                "content": "You are a helpful AI assistant for the Physical AI & Humanoid Robotics book. Answer questions based only on the provided context."
            },
            {
                "role": "user",
                "content": f"""
Answer ONLY using the content below.

CONTENT:
{context}

QUESTION:
What is ROS 2?

ANSWER:
"""
            }
        ],
        max_tokens=250,
    )

    answer = chat_response.choices[0].message.content.strip()
    print(f"V Chat response received:")
    print(f"Answer: {answer.encode('ascii', errors='ignore').decode('ascii')}")

except Exception as e:
    print(f"Error in pipeline: {e}")
    import traceback
    traceback.print_exc()