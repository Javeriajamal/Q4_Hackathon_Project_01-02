#!/usr/bin/env python3
"""
Test script to verify Qdrant query_points functionality
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

# Test embedding and query
try:
    print("Testing embedding and query_points...")

    # Generate embedding for test query
    response = openai_client.embeddings.create(
        model="BAAI/bge-large-en-v1.5",
        input=["What is ROS 2?"]
    )

    query_embedding = response.data[0].embedding
    print(f"Query embedding generated with dimension: {len(query_embedding)}")

    # Perform query in Qdrant using the query_points method
    results = qdrant_client.query_points(
        collection_name="rag_embedding",
        query=query_embedding,
        limit=3
    )

    # The results might be a different type, let's check
    print(f"Results type: {type(results)}")

    # Check if it's a list-like object or has a different structure
    if hasattr(results, '__dict__'):
        print(f"Results attributes: {dir(results)}")

    # Try to access the points in the results
    # In newer versions, query_points might return a different structure
    if hasattr(results, 'points'):
        points = results.points
        print(f"Found {len(points)} points:")
        for i, point in enumerate(points):
            print(f"Result {i+1}:")
            print(f"  Score: {getattr(point, 'score', 'N/A')}")
            print(f"  Text preview: {point.payload.get('text', '')[:100]}...")
            print(f"  URL: {point.payload.get('url', '')}")
            print()
    else:
        # If it's a list directly
        try:
            points = list(results)
            print(f"Found {len(points)} points:")
            for i, point in enumerate(points):
                print(f"Result {i+1}:")
                print(f"  Score: {getattr(point, 'score', 'N/A')}")
                print(f"  Text preview: {point.payload.get('text', '')[:100]}...")
                print(f"  URL: {point.payload.get('url', '')}")
                print()
        except:
            print("Could not access results as expected format")

    # Test if results are empty
    if hasattr(results, 'points') and len(results.points) == 0:
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
    print(f"Error in query test: {e}")
    import traceback
    traceback.print_exc()