#!/usr/bin/env python3
"""
Test script to verify Qdrant collection and connectivity
"""
import os
from qdrant_client import QdrantClient
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Initialize Qdrant client
qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)

# Check if collection exists and get info
try:
    collection_info = qdrant_client.get_collection("rag_embedding")
    print(f"Collection 'rag_embedding' exists!")
    print(f"Points count: {collection_info.points_count}")
    print(f"Vector size: {collection_info.config.params.vectors.size}")
    print(f"Distance: {collection_info.config.params.vectors.distance}")

    # Try to get a few sample points
    sample_points = qdrant_client.scroll(
        collection_name="rag_embedding",
        limit=3
    )

    print("\nSample points:")
    for i, point in enumerate(sample_points[0]):
        print(f"Point {i+1}: ID={point.id}")
        print(f"  Text preview: {point.payload.get('text', '')[:100]}...")
        print(f"  URL: {point.payload.get('url', '')}")
        print()

except Exception as e:
    print(f"Error accessing Qdrant: {e}")