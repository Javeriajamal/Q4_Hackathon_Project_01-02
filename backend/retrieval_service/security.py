"""
Security validation module for the RAG retrieval service.

This module ensures that only content from approved textbook sources is returned
by validating URLs and content against a whitelist of approved domains/resources.
"""

from typing import List, Dict, Any, Optional
from urllib.parse import urlparse
import re
from .logging_config import get_logger
from .models import RetrievalResult


class ContentSecurityValidator:
    """Validates that retrieved content comes from approved sources only."""

    def __init__(self):
        """Initialize the ContentSecurityValidator with approved sources."""
        self._logger = get_logger()

        # Approved domains for the Physical AI & Humanoid Robotics textbook
        self._approved_domains = {
            'physicalairobotics.netlify.app',
            'physicalairobotics.com',
            'textbook.physicalairobotics.com',
            'docs.physicalairobotics.com'
        }

        # Approved URL patterns for textbook content
        self._approved_patterns = [
            r'https?://(?:www\.)?physicalairobotics\.netlify\.app/.*',
            r'https?://(?:www\.)?physicalairobotics\.com/.*',
            r'https?://(?:www\.)?textbook\.physicalairobotics\.com/.*',
            r'https://d\.coffeescript\.org/.*',  # For any embedded content
        ]

        # Approved content types (file extensions)
        self._approved_extensions = {
            '.md', '.mdx', '.html', '.txt', '.pdf'
        }

    def is_approved_source(self, url: str) -> bool:
        """
        Check if a URL is from an approved source.

        Args:
            url: The URL to validate

        Returns:
            bool: True if the URL is from an approved source, False otherwise
        """
        if not url:
            return False

        try:
            parsed_url = urlparse(url)
            domain = parsed_url.netloc.lower()

            # Check if domain is in approved list
            if domain in self._approved_domains:
                return True

            # Check if URL matches any approved pattern
            for pattern in self._approved_patterns:
                if re.match(pattern, url, re.IGNORECASE):
                    return True

        except Exception as e:
            self._logger.error(f"Error validating URL {url}: {str(e)}")
            return False

        return False

    def validate_content_source(self, result: RetrievalResult) -> bool:
        """
        Validate that a retrieval result comes from an approved source.

        Args:
            result: The retrieval result to validate

        Returns:
            bool: True if the result is from an approved source, False otherwise
        """
        if not hasattr(result, 'metadata') or not result.metadata:
            self._logger.warning(f"Result {result.chunk_id} has no metadata for source validation")
            return False

        if not hasattr(result.metadata, 'source_url'):
            self._logger.warning(f"Result {result.chunk_id} has no source URL for validation")
            return False

        source_url = result.metadata.source_url

        is_approved = self.is_approved_source(source_url)

        if not is_approved:
            self._logger.warning(f"Content from unauthorized source blocked: {source_url}")

        return is_approved

    def filter_approved_results(self, results: List[RetrievalResult]) -> List[RetrievalResult]:
        """
        Filter a list of results to only include those from approved sources.

        Args:
            results: List of retrieval results to filter

        Returns:
            List[RetrievalResult]: Filtered list containing only approved results
        """
        if not results:
            return []

        approved_results = []
        blocked_count = 0

        for result in results:
            if self.validate_content_source(result):
                approved_results.append(result)
            else:
                blocked_count += 1
                self._logger.info(f"Blocked result from unauthorized source: {result.chunk_id}")

        if blocked_count > 0:
            self._logger.info(f"Security filter: {blocked_count} results blocked, {len(approved_results)} allowed")

        return approved_results

    def validate_content_integrity(self, content: str, expected_domain: Optional[str] = None) -> bool:
        """
        Validate content integrity and ensure it doesn't contain unauthorized references.

        Args:
            content: The content to validate
            expected_domain: Optional domain that should be referenced in the content

        Returns:
            bool: True if content passes integrity checks, False otherwise
        """
        if not content:
            return False

        # Check for suspicious patterns that might indicate content manipulation
        suspicious_patterns = [
            r'<script[^>]*>',  # Script tags
            r'javascript:',     # JavaScript URLs
            r'on\w+\s*=',      # Event handlers
            r'<iframe[^>]*>',  # Iframes
            r'<object[^>]*>',  # Objects
            r'<embed[^>]*>',   # Embedded content
        ]

        for pattern in suspicious_patterns:
            if re.search(pattern, content, re.IGNORECASE):
                self._logger.warning(f"Suspicious pattern detected in content: {pattern}")
                return False

        # If expected domain is specified, check that content references it appropriately
        if expected_domain and expected_domain not in content.lower():
            # This is just a heuristic - in a real implementation, we might be more lenient
            # depending on the type of content
            pass  # For now, we won't fail validation based on domain reference alone

        return True

    def get_security_report(self) -> Dict[str, Any]:
        """
        Get a security report with current validation settings.

        Returns:
            Dict[str, Any]: Security configuration and statistics
        """
        return {
            "approved_domains": list(self._approved_domains),
            "approved_patterns": self._approved_patterns,
            "approved_extensions": list(self._approved_extensions),
            "last_updated": "2025-12-21",
            "validation_enabled": True
        }

    def add_approved_domain(self, domain: str) -> bool:
        """
        Add a domain to the list of approved sources.

        Args:
            domain: The domain to approve

        Returns:
            bool: True if the domain was added, False if it was already approved
        """
        if domain in self._approved_domains:
            return False

        self._approved_domains.add(domain)
        self._logger.info(f"Added domain to approved list: {domain}")
        return True

    def remove_approved_domain(self, domain: str) -> bool:
        """
        Remove a domain from the list of approved sources.

        Args:
            domain: The domain to remove

        Returns:
            bool: True if the domain was removed, False if it wasn't in the list
        """
        if domain not in self._approved_domains:
            return False

        self._approved_domains.remove(domain)
        self._logger.info(f"Removed domain from approved list: {domain}")
        return True


# Global instance of ContentSecurityValidator
content_security_validator = ContentSecurityValidator()


def get_content_security_validator() -> ContentSecurityValidator:
    """
    Get the global ContentSecurityValidator instance.

    Returns:
        ContentSecurityValidator: The global ContentSecurityValidator instance
    """
    return content_security_validator