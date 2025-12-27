#!/usr/bin/env python3
"""
Test script to verify Qdrant search functionality - using exact same approach as main.py
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
    print("Testing embedding and search (using exact same approach as main.py)...")

    # Generate embedding for test query
    response = openai_client.embeddings.create(
        model="BAAI/bge-large-en-v1.5",
        input=["What is the Physical AI & Humanoid Robotics book about?"]
    )

    query_embedding = response.data[0].embedding
    print(f"Query embedding generated with dimension: {len(query_embedding)}")

    # Perform search in Qdrant using the exact same method as main.py
    results = qdrant_client.search_points(
        collection_name="rag_embedding",
        query=query_embedding,  # Parameter name might be 'query' instead of 'vector'
        limit=3
    )

    print(f"Found {len(results)} results:")
    for i, result in enumerate(results):
        print(f"Result {i+1}:")
        print(f"  Score: {result.score}")
        print(f"  Text preview: {result.payload.get('text', '')[:100]}...")
        print(f"  URL: {result.payload.get('url', '')}")
        print()

except Exception as e:
    print(f"Error in search test: {e}")
    import traceback
    traceback.print_exc()

    # Try alternative method name or parameter
    try:
        print("\nTrying alternative method...")
        results = qdrant_client.search(
            collection_name="rag_embedding",
            query_vector=query_embedding,
            limit=3
        )

        print(f"Alternative method worked! Found {len(results)} results:")
        for i, result in enumerate(results):
            print(f"Result {i+1}:")
            print(f"  Score: {result.score}")
            print(f"  Text preview: {result.payload.get('text', '')[:100]}...")
            print(f"  URL: {result.payload.get('url', '')}")
            print()
    except Exception as e2:
        print(f"Alternative method also failed: {e2}")