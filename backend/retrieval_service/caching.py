"""
Caching functionality for the RAG retrieval service.

This module implements a caching layer to improve response times for frequently requested queries.
"""

import hashlib
import json
import time
from typing import Dict, Optional, Any, Tuple
from datetime import datetime, timedelta
from functools import wraps
from threading import Lock

from .models import RetrievalResult
from .logging_config import get_logger


class QueryCache:
    """In-memory cache for retrieval queries to improve response times."""

    def __init__(self, max_size: int = 1000, ttl_seconds: int = 3600):
        """
        Initialize the QueryCache.

        Args:
            max_size: Maximum number of cached items
            ttl_seconds: Time-to-live for cached items in seconds
        """
        self._cache: Dict[str, Tuple[Dict[str, Any], float]] = {}  # {hash: (result, expiry_time)}
        self._max_size = max_size
        self._ttl_seconds = ttl_seconds
        self._lock = Lock()  # Thread safety for cache operations
        self._logger = get_logger()
        self._hits = 0
        self._misses = 0

    def _generate_cache_key(self, query: str, top_k: int, min_relevance: float) -> str:
        """
        Generate a unique cache key for a query.

        Args:
            query: The query string
            top_k: Number of results requested
            min_relevance: Minimum relevance threshold

        Returns:
            str: Hashed cache key
        """
        cache_input = f"{query}:{top_k}:{min_relevance}"
        return hashlib.sha256(cache_input.encode()).hexdigest()

    def get(self, query: str, top_k: int, min_relevance: float) -> Optional[Dict[str, Any]]:
        """
        Get a cached result if available and not expired.

        Args:
            query: The query string
            top_k: Number of results requested
            min_relevance: Minimum relevance threshold

        Returns:
            Optional[Dict[str, Any]]: Cached result or None if not available/expired
        """
        with self._lock:
            cache_key = self._generate_cache_key(query, top_k, min_relevance)

            if cache_key in self._cache:
                result, expiry_time = self._cache[cache_key]

                if time.time() < expiry_time:
                    self._hits += 1
                    hit_rate = self._hits / (self._hits + self._misses) * 100
                    self._logger.info(f"Cache HIT for query: '{query[:50]}...', Hit rate: {hit_rate:.2f}%")
                    return result
                else:
                    # Entry expired, remove it
                    del self._cache[cache_key]
                    self._logger.info(f"Cache EXPIRED entry removed for query: '{query[:50]}...'")

            self._misses += 1
            hit_rate = self._hits / (self._hits + self._misses) * 100
            self._logger.info(f"Cache MISS for query: '{query[:50]}...', Hit rate: {hit_rate:.2f}%")
            return None

    def set(self, query: str, top_k: int, min_relevance: float, result: Dict[str, Any]) -> None:
        """
        Store a result in the cache.

        Args:
            query: The query string
            top_k: Number of results requested
            min_relevance: Minimum relevance threshold
            result: The result to cache
        """
        with self._lock:
            cache_key = self._generate_cache_key(query, top_k, min_relevance)
            expiry_time = time.time() + self._ttl_seconds

            # If cache is at max capacity, remove oldest entry (simple FIFO)
            if len(self._cache) >= self._max_size:
                # Remove the first item in the dictionary (oldest in a simple implementation)
                # In a production system, we might want LRU eviction
                oldest_key = next(iter(self._cache))
                del self._cache[oldest_key]
                self._logger.info("Cache EVICTION: Removed oldest entry due to size limit")

            self._cache[cache_key] = (result, expiry_time)
            self._logger.info(f"Cache SET for query: '{query[:50]}...', Cache size: {len(self._cache)}")

    def invalidate(self, query: Optional[str] = None) -> int:
        """
        Invalidate specific cache entry or all entries.

        Args:
            query: Specific query to invalidate (None to clear all)

        Returns:
            int: Number of entries invalidated
        """
        with self._lock:
            if query is None:
                count = len(self._cache)
                self._cache.clear()
                self._logger.info(f"Cache CLEAR: All {count} entries invalidated")
                return count
            else:
                # Invalidate based on query pattern (would need to scan keys in a real implementation)
                # For simplicity, we'll just clear all - in production, we might have tags or patterns
                count = len(self._cache)
                self._cache.clear()
                self._logger.info(f"Cache INVALIDATE: All entries cleared (pattern-based invalidation not implemented)")
                return count

    def get_stats(self) -> Dict[str, Any]:
        """
        Get cache statistics.

        Returns:
            Dict[str, Any]: Cache statistics
        """
        total_requests = self._hits + self._misses
        hit_rate = (self._hits / total_requests * 100) if total_requests > 0 else 0.0

        return {
            "cache_size": len(self._cache),
            "max_size": self._max_size,
            "ttl_seconds": self._ttl_seconds,
            "hits": self._hits,
            "misses": self._misses,
            "total_requests": total_requests,
            "hit_rate_percent": round(hit_rate, 2),
            "cached_entries": list(self._cache.keys())  # Just the keys for security
        }

    def cleanup_expired(self) -> int:
        """
        Remove expired entries from the cache.

        Returns:
            int: Number of expired entries removed
        """
        with self._lock:
            current_time = time.time()
            expired_keys = []

            for key, (_, expiry_time) in self._cache.items():
                if current_time >= expiry_time:
                    expired_keys.append(key)

            for key in expired_keys:
                del self._cache[key]

            if expired_keys:
                self._logger.info(f"Cache CLEANUP: Removed {len(expired_keys)} expired entries")

            return len(expired_keys)


class CachingDecorator:
    """Decorator to add caching functionality to retrieval methods."""

    def __init__(self, cache_ttl: int = 3600):
        """
        Initialize the CachingDecorator.

        Args:
            cache_ttl: Time-to-live for cached items in seconds
        """
        self._cache = QueryCache(ttl_seconds=cache_ttl)
        self._logger = get_logger()

    def cache_retrieval_method(self, func):
        """
        Decorator to cache retrieval method results.

        Args:
            func: The function to decorate

        Returns:
            The decorated function
        """
        @wraps(func)
        def wrapper(*args, **kwargs):
            # Extract query parameters for cache key
            query = kwargs.get('query') or (args[0] if args else None)
            top_k = kwargs.get('top_k', 5)
            min_relevance = kwargs.get('min_relevance', 0.3)

            if not query:
                self._logger.warning("Skipping cache - no query provided")
                return func(*args, **kwargs)

            # Try to get from cache first
            cached_result = self._cache.get(query, top_k, min_relevance)
            if cached_result is not None:
                self._logger.info(f"Returning cached result for query: '{query[:50]}...'")
                return cached_result

            # Execute the original function
            start_time = time.time()
            result = func(*args, **kwargs)
            execution_time = time.time() - start_time

            # Cache the result if it was a successful retrieval
            if result and isinstance(result, dict) and 'results' in result:
                # Add cache-specific metadata
                result['from_cache'] = False
                result['cache_miss_reason'] = 'fresh_retrieval'
                result['original_execution_time'] = execution_time

                # Only cache if execution was not extremely fast (suggesting it was already cached somewhere else)
                if execution_time > 0.01:  # Only cache if took more than 10ms to retrieve
                    self._cache.set(query, top_k, min_relevance, result)
                    self._logger.info(f"Cached new result for query: '{query[:50]}...' (took {execution_time:.3f}s)")

            return result

        return wrapper

    def get_cache(self) -> QueryCache:
        """
        Get the underlying cache instance.

        Returns:
            QueryCache: The cache instance
        """
        return self._cache


# Global instance of QueryCache
query_cache = QueryCache(max_size=1000, ttl_seconds=3600)  # 1000 items, 1 hour TTL


def get_query_cache() -> QueryCache:
    """
    Get the global QueryCache instance.

    Returns:
        QueryCache: The global QueryCache instance
    """
    return query_cache


# Global instance of CachingDecorator
caching_decorator = CachingDecorator(cache_ttl=3600)


def get_caching_decorator() -> CachingDecorator:
    """
    Get the global CachingDecorator instance.

    Returns:
        CachingDecorator: The global CachingDecorator instance
    """
    return caching_decorator


def cache_retrieval_endpoint(func):
    """
    Decorator to add caching to a retrieval endpoint function.

    Args:
        func: The endpoint function to decorate

    Returns:
        The decorated function with caching functionality
    """
    return caching_decorator.cache_retrieval_method(func)


# Example usage in a service class
class CachedRetrievalService:
    """Example service class demonstrating cache integration."""

    def __init__(self):
        self._cache = get_query_cache()
        self._logger = get_logger()

    def retrieve_with_cache(
        self,
        query: str,
        top_k: int = 5,
        min_relevance: float = 0.3
    ) -> Dict[str, Any]:
        """
        Retrieve content with automatic caching.

        Args:
            query: The query string
            top_k: Number of results to return
            min_relevance: Minimum relevance threshold

        Returns:
            Dict[str, Any]: Retrieval results
        """
        # Try cache first
        cached_result = self._cache.get(query, top_k, min_relevance)
        if cached_result:
            cached_result['from_cache'] = True
            self._logger.info(f"Serving from cache: '{query[:50]}...'")
            return cached_result

        # If not in cache, perform actual retrieval
        # (In a real implementation, this would call the actual retrieval logic)
        results = self._perform_actual_retrieval(query, top_k, min_relevance)

        # Format the result
        result_data = {
            "results": results,
            "query": query,
            "top_k": top_k,
            "min_relevance": min_relevance,
            "from_cache": False,
            "timestamp": datetime.now().isoformat()
        }

        # Cache the result
        self._cache.set(query, top_k, min_relevance, result_data)
        self._logger.info(f"Stored in cache: '{query[:50]}...'")

        return result_data

    def _perform_actual_retrieval(self, query: str, top_k: int, min_relevance: float) -> List[Dict[str, Any]]:
        """
        Perform the actual retrieval logic (placeholder implementation).

        Args:
            query: The query string
            top_k: Number of results to return
            min_relevance: Minimum relevance threshold

        Returns:
            List[Dict[str, Any]]: List of result dictionaries
        """
        # This is a placeholder - in real implementation, this would call
        # the vector search, ranking, and formatting logic
        return [
            {
                "chunk_id": f"demo_chunk_{i}",
                "content": f"Demo content for query '{query}' - result {i}",
                "relevance_score": 0.9 - (i * 0.1),
                "metadata": {
                    "source_url": "https://physicalairobotics.netlify.app/demo",
                    "section_title": "Demo Section",
                    "chunk_index": i
                },
                "rank": i + 1
            }
            for i in range(min(top_k, 5))  # Limit to 5 for demo
        ]

    def invalidate_cache_for_query(self, query: str) -> bool:
        """
        Invalidate cache entries related to a specific query.

        Args:
            query: The query to invalidate cache for

        Returns:
            bool: True if entries were invalidated, False otherwise
        """
        # In a simple implementation, we'll clear the entire cache
        # In production, we might implement pattern-based invalidation
        invalidated_count = self._cache.invalidate(query)
        self._logger.info(f"Invalidated {invalidated_count} cache entries for query patterns related to: '{query[:50]}...'")
        return invalidated_count > 0

    def get_cache_stats(self) -> Dict[str, Any]:
        """
        Get cache statistics.

        Returns:
            Dict[str, Any]: Cache statistics
        """
        return self._cache.get_stats()

    def refresh_cache_entry(self, query: str, top_k: int = 5, min_relevance: float = 0.3) -> bool:
        """
        Refresh a cache entry by re-executing the query and updating the cache.

        Args:
            query: The query to refresh
            top_k: Number of results to return
            min_relevance: Minimum relevance threshold

        Returns:
            bool: True if refreshed successfully, False otherwise
        """
        try:
            # Remove the current cached entry
            # For simplicity, we'll clear all cache - in production we'd target specific entries
            self._cache.invalidate(query)

            # Execute the query again to repopulate cache
            result = self.retrieve_with_cache(query, top_k, min_relevance)

            self._logger.info(f"Refreshed cache entry for query: '{query[:50]}...'")
            return True
        except Exception as e:
            self._logger.error(f"Error refreshing cache for query '{query[:50]}...': {str(e)}")
            return False


# Example of how to use caching in the API layer
def apply_caching_to_retrieval(retrieval_func):
    """
    Apply caching to a retrieval function.

    Args:
        retrieval_func: Function that performs retrieval

    Returns:
        Function with caching applied
    """
    @wraps(retrieval_func)
    def cached_version(*args, **kwargs):
        # Extract query parameters from function arguments
        # This is a simplified version - actual implementation would depend on function signature
        query = kwargs.get('query') or (args[1] if len(args) > 1 else args[0] if args else None)

        if not query:
            return retrieval_func(*args, **kwargs)

        # Get cache instance
        cache = get_query_cache()

        # Generate cache key based on query and parameters
        top_k = kwargs.get('top_k', 5)
        min_relevance = kwargs.get('min_relevance', 0.3)

        # Check cache first
        cached_result = cache.get(query, top_k, min_relevance)
        if cached_result:
            return cached_result

        # Execute original function
        result = retrieval_func(*args, **kwargs)

        # Cache the result
        cache.set(query, top_k, min_relevance, result)

        return result

    return cached_version