"""
Monitoring and alerting functionality for the RAG retrieval service.

This module provides system health monitoring, performance tracking, and alerting capabilities.
"""

import time
import psutil
import GPUtil
from datetime import datetime, timedelta
from typing import Dict, List, Any, Optional
from dataclasses import dataclass, asdict
from enum import Enum
import smtplib
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart

from .logging_config import get_logger


class AlertSeverity(Enum):
    """Enumeration for alert severity levels."""
    LOW = "low"
    MEDIUM = "medium"
    HIGH = "high"
    CRITICAL = "critical"


class AlertType(Enum):
    """Enumeration for alert types."""
    PERFORMANCE = "performance"
    AVAILABILITY = "availability"
    RESOURCE_UTILIZATION = "resource_utilization"
    ERROR_RATE = "error_rate"
    DATA_INTEGRITY = "data_integrity"


@dataclass
class SystemMetrics:
    """Data class to hold system metrics."""
    timestamp: datetime
    cpu_percent: float
    memory_percent: float
    disk_percent: float
    gpu_percent: Optional[float]
    gpu_memory_percent: Optional[float]
    active_processes: int
    network_io: Dict[str, float]  # bytes sent/received per second
    response_time_p50: float
    response_time_p95: float
    response_time_p99: float
    requests_per_minute: float
    error_rate: float
    active_connections: int


@dataclass
class Alert:
    """Data class to hold alert information."""
    alert_id: str
    timestamp: datetime
    severity: AlertSeverity
    alert_type: AlertType
    message: str
    details: Dict[str, Any]
    resolved: bool = False
    resolved_at: Optional[datetime] = None


class MonitoringService:
    """Main monitoring service that collects metrics and manages alerts."""

    def __init__(self):
        """Initialize the MonitoringService."""
        self._logger = get_logger()
        self._alerts: List[Alert] = []
        self._metrics_history: List[SystemMetrics] = []
        self._performance_thresholds = {
            'response_time_p95': 2.0,  # 2 seconds
            'error_rate': 0.05,        # 5%
            'cpu_percent': 80.0,       # 80%
            'memory_percent': 85.0,    # 85%
            'disk_percent': 90.0       # 90%
        }

        # Alert configuration
        self._email_config = {
            'smtp_server': 'smtp.gmail.com',
            'smtp_port': 587,
            'sender_email': 'alert@physicalairobotics.com',
            'sender_password': 'APP_PASSWORD_HERE',  # Use app password, not account password
            'recipient_emails': ['admin@physicalairobotics.com']
        }

        self._logger.info("Monitoring service initialized")

    def collect_system_metrics(self) -> SystemMetrics:
        """
        Collect current system metrics.

        Returns:
            SystemMetrics: Current system metrics
        """
        # CPU utilization
        cpu_percent = psutil.cpu_percent(interval=1)

        # Memory utilization
        memory = psutil.virtual_memory()
        memory_percent = memory.percent

        # Disk utilization
        disk = psutil.disk_usage('/')
        disk_percent = (disk.used / disk.total) * 100

        # GPU utilization (if available)
        gpu_percent = None
        gpu_memory_percent = None
        gpus = GPUtil.getGPUs()
        if gpus:
            gpu = gpus[0]  # Use first GPU
            gpu_percent = gpu.load * 100
            gpu_memory_percent = gpu.memoryUtil * 100

        # Active processes
        active_processes = len([p for p in psutil.process_iter(['pid', 'name', 'status'])])

        # Network I/O (simplified - would need more complex tracking in real system)
        net_io = psutil.net_io_counters()
        network_io = {
            'bytes_sent_per_sec': 0,  # Would need to track over time
            'bytes_recv_per_sec': 0
        }

        # Performance metrics (these would be tracked from actual service operations)
        # Using placeholder values - in real system these would come from actual measurements
        response_time_p50 = 0.5  # seconds
        response_time_p95 = 1.2  # seconds
        response_time_p99 = 1.8  # seconds
        requests_per_minute = 100.0  # placeholder
        error_rate = 0.02  # 2% error rate
        active_connections = 10  # placeholder

        metrics = SystemMetrics(
            timestamp=datetime.now(),
            cpu_percent=cpu_percent,
            memory_percent=memory_percent,
            disk_percent=disk_percent,
            gpu_percent=gpu_percent,
            gpu_memory_percent=gpu_memory_percent,
            active_processes=active_processes,
            network_io=network_io,
            response_time_p50=response_time_p50,
            response_time_p95=response_time_p95,
            response_time_p99=response_time_p99,
            requests_per_minute=requests_per_minute,
            error_rate=error_rate,
            active_connections=active_connections
        )

        # Store metrics in history (keep last 1000 entries)
        self._metrics_history.append(metrics)
        if len(self._metrics_history) > 1000:
            self._metrics_history.pop(0)

        self._logger.debug(f"Collected system metrics at {metrics.timestamp}")
        return metrics

    def check_thresholds(self, metrics: SystemMetrics) -> List[Alert]:
        """
        Check if any metrics exceed defined thresholds and create alerts.

        Args:
            metrics: Current system metrics to check

        Returns:
            List[Alert]: List of new alerts generated
        """
        new_alerts = []

        # Check response time thresholds
        if metrics.response_time_p95 > self._performance_thresholds['response_time_p95']:
            alert = Alert(
                alert_id=f"perf_{int(datetime.now().timestamp())}",
                timestamp=datetime.now(),
                severity=AlertSeverity.HIGH,
                alert_type=AlertType.PERFORMANCE,
                message=f"P95 Response time ({metrics.response_time_p95}s) exceeded threshold ({self._performance_thresholds['response_time_p95']}s)",
                details={
                    'current_value': metrics.response_time_p95,
                    'threshold': self._performance_thresholds['response_time_p95'],
                    'metric_type': 'response_time_p95'
                }
            )
            new_alerts.append(alert)

        # Check error rate
        if metrics.error_rate > self._performance_thresholds['error_rate']:
            alert = Alert(
                alert_id=f"error_{int(datetime.now().timestamp())}",
                timestamp=datetime.now(),
                severity=AlertSeverity.HIGH,
                alert_type=AlertType.ERROR_RATE,
                message=f"Error rate ({metrics.error_rate:.2%}) exceeded threshold ({self._performance_thresholds['error_rate']:.2%})",
                details={
                    'current_value': metrics.error_rate,
                    'threshold': self._performance_thresholds['error_rate'],
                    'metric_type': 'error_rate'
                }
            )
            new_alerts.append(alert)

        # Check CPU utilization
        if metrics.cpu_percent > self._performance_thresholds['cpu_percent']:
            alert = Alert(
                alert_id=f"cpu_{int(datetime.now().timestamp())}",
                timestamp=datetime.now(),
                severity=AlertSeverity.MEDIUM,
                alert_type=AlertType.RESOURCE_UTILIZATION,
                message=f"CPU utilization ({metrics.cpu_percent}%) exceeded threshold ({self._performance_thresholds['cpu_percent']}%)",
                details={
                    'current_value': metrics.cpu_percent,
                    'threshold': self._performance_thresholds['cpu_percent'],
                    'metric_type': 'cpu_percent'
                }
            )
            new_alerts.append(alert)

        # Check memory utilization
        if metrics.memory_percent > self._performance_thresholds['memory_percent']:
            alert = Alert(
                alert_id=f"mem_{int(datetime.now().timestamp())}",
                timestamp=datetime.now(),
                severity=AlertSeverity.MEDIUM,
                alert_type=AlertType.RESOURCE_UTILIZATION,
                message=f"Memory utilization ({metrics.memory_percent}%) exceeded threshold ({self._performance_thresholds['memory_percent']}%)",
                details={
                    'current_value': metrics.memory_percent,
                    'threshold': self._performance_thresholds['memory_percent'],
                    'metric_type': 'memory_percent'
                }
            )
            new_alerts.append(alert)

        # Check disk utilization
        if metrics.disk_percent > self._performance_thresholds['disk_percent']:
            alert = Alert(
                alert_id=f"disk_{int(datetime.now().timestamp())}",
                timestamp=datetime.now(),
                severity=AlertSeverity.HIGH,
                alert_type=AlertType.RESOURCE_UTILIZATION,
                message=f"Disk utilization ({metrics.disk_percent}%) exceeded threshold ({self._performance_thresholds['disk_percent']}%)",
                details={
                    'current_value': metrics.disk_percent,
                    'threshold': self._performance_thresholds['disk_percent'],
                    'metric_type': 'disk_percent'
                }
            )
            new_alerts.append(alert)

        # Add new alerts to our collection
        for alert in new_alerts:
            self._alerts.append(alert)
            self._logger.warning(f"ALERT GENERATED [{alert.severity.value.upper()}]: {alert.message}")

        return new_alerts

    def send_alert_notification(self, alert: Alert) -> bool:
        """
        Send alert notification via email.

        Args:
            alert: The alert to notify about

        Returns:
            bool: True if notification was sent successfully, False otherwise
        """
        try:
            # Create message
            msg = MIMEMultipart()
            msg['From'] = self._email_config['sender_email']
            msg['To'] = ', '.join(self._email_config['recipient_emails'])
            msg['Subject'] = f"[{alert.severity.value.upper()}] {alert.alert_type.value.title()} Alert"

            # Create message body
            body = f"""
            Alert ID: {alert.alert_id}
            Timestamp: {alert.timestamp.isoformat()}
            Severity: {alert.severity.value.upper()}
            Type: {alert.alert_type.value.replace('_', ' ').title()}

            Message: {alert.message}

            Details:
            {json.dumps(alert.details, indent=2)}

            Please investigate and take appropriate action.
            """

            msg.attach(MIMEText(body, 'plain'))

            # Connect to server and send email
            server = smtplib.SMTP(self._email_config['smtp_server'], self._email_config['smtp_port'])
            server.starttls()
            server.login(self._email_config['sender_email'], self._email_config['sender_password'])

            text = msg.as_string()
            server.sendmail(
                self._email_config['sender_email'],
                self._email_config['recipient_emails'],
                text
            )
            server.quit()

            self._logger.info(f"Alert notification sent successfully: {alert.alert_id}")
            return True

        except Exception as e:
            self._logger.error(f"Failed to send alert notification: {str(e)}")
            return False

    def get_system_health_status(self) -> Dict[str, Any]:
        """
        Get overall system health status.

        Returns:
            Dict[str, Any]: System health status information
        """
        current_metrics = self.collect_system_metrics()
        active_alerts = [alert for alert in self._alerts if not alert.resolved]

        # Determine overall health status
        if any(alert.severity == AlertSeverity.CRITICAL for alert in active_alerts):
            status = "critical"
        elif any(alert.severity in [AlertSeverity.HIGH, AlertSeverity.CRITICAL] for alert in active_alerts):
            status = "warning"
        else:
            status = "healthy"

        health_status = {
            "status": status,
            "timestamp": datetime.now().isoformat(),
            "current_metrics": asdict(current_metrics),
            "active_alerts_count": len(active_alerts),
            "recent_alerts": [
                {
                    "id": alert.alert_id,
                    "severity": alert.severity.value,
                    "type": alert.alert_type.value,
                    "message": alert.message,
                    "timestamp": alert.timestamp.isoformat()
                }
                for alert in active_alerts[-5:]  # Last 5 alerts
            ],
            "system_uptime_minutes": self._calculate_uptime_minutes(),
            "performance_trend": self._analyze_performance_trend()
        }

        return health_status

    def _calculate_uptime_minutes(self) -> float:
        """
        Calculate system uptime in minutes.
        This is a simplified version - in production would track from service start.

        Returns:
            float: Uptime in minutes
        """
        # In a real implementation, this would track from when the service started
        # For now, returning a placeholder
        return 1440.0  # 24 hours

    def _analyze_performance_trend(self) -> str:
        """
        Analyze performance trend based on recent metrics.

        Returns:
            str: Performance trend ('improving', 'stable', 'declining')
        """
        if len(self._metrics_history) < 2:
            return "insufficient_data"

        # Look at last 10 metrics for trend analysis
        recent_metrics = self._metrics_history[-10:]

        # Analyze response time trend
        response_times = [m.response_time_p95 for m in recent_metrics if m.response_time_p95 is not None]
        if len(response_times) < 2:
            return "insufficient_data"

        # Simple linear trend analysis
        first_rt = response_times[0]
        last_rt = response_times[-1]

        if last_rt > first_rt * 1.1:  # Increased by more than 10%
            return "declining"
        elif last_rt < first_rt * 0.9:  # Decreased by more than 10%
            return "improving"
        else:
            return "stable"

    def resolve_alert(self, alert_id: str) -> bool:
        """
        Mark an alert as resolved.

        Args:
            alert_id: ID of the alert to resolve

        Returns:
            bool: True if alert was found and resolved, False otherwise
        """
        for alert in self._alerts:
            if alert.alert_id == alert_id and not alert.resolved:
                alert.resolved = True
                alert.resolved_at = datetime.now()
                self._logger.info(f"Alert {alert_id} marked as resolved")
                return True

        self._logger.warning(f"Could not find alert {alert_id} to resolve")
        return False

    def get_metrics_summary(self, hours: int = 24) -> Dict[str, Any]:
        """
        Get a summary of metrics for the specified time period.

        Args:
            hours: Number of hours to include in the summary

        Returns:
            Dict[str, Any]: Metrics summary
        """
        cutoff_time = datetime.now() - timedelta(hours=hours)
        relevant_metrics = [m for m in self._metrics_history if m.timestamp >= cutoff_time]

        if not relevant_metrics:
            return {
                "period_hours": hours,
                "sample_count": 0,
                "metrics_available": False
            }

        # Calculate averages
        avg_cpu = sum(m.cpu_percent for m in relevant_metrics) / len(relevant_metrics)
        avg_memory = sum(m.memory_percent for m in relevant_metrics) / len(relevant_metrics)
        avg_response_p50 = sum(m.response_time_p50 for m in relevant_metrics if m.response_time_p50 is not None) / len([m for m in relevant_metrics if m.response_time_p50 is not None])
        avg_response_p95 = sum(m.response_time_p95 for m in relevant_metrics if m.response_time_p95 is not None) / len([m for m in relevant_metrics if m.response_time_p95 is not None])
        avg_error_rate = sum(m.error_rate for m in relevant_metrics) / len(relevant_metrics)
        avg_requests_per_minute = sum(m.requests_per_minute for m in relevant_metrics) / len(relevant_metrics)

        # Calculate peak values
        peak_cpu = max(m.cpu_percent for m in relevant_metrics)
        peak_memory = max(m.memory_percent for m in relevant_metrics)
        peak_response_p95 = max(m.response_time_p95 for m in relevant_metrics if m.response_time_p95 is not None)

        # Count alerts in period
        period_alerts = [a for a in self._alerts if a.timestamp >= cutoff_time]

        return {
            "period_hours": hours,
            "sample_count": len(relevant_metrics),
            "metrics_available": True,
            "averages": {
                "cpu_percent": round(avg_cpu, 2),
                "memory_percent": round(avg_memory, 2),
                "response_time_p50": round(avg_response_p50, 3),
                "response_time_p95": round(avg_response_p95, 3),
                "error_rate": round(avg_error_rate, 4),
                "requests_per_minute": round(avg_requests_per_minute, 2)
            },
            "peaks": {
                "cpu_percent": peak_cpu,
                "memory_percent": peak_memory,
                "response_time_p95": peak_response_p95
            },
            "alert_counts": {
                "total": len(period_alerts),
                "critical": len([a for a in period_alerts if a.severity == AlertSeverity.CRITICAL]),
                "high": len([a for a in period_alerts if a.severity == AlertSeverity.HIGH]),
                "medium": len([a for a in period_alerts if a.severity == AlertSeverity.MEDIUM]),
                "low": len([a for a in period_alerts if a.severity == AlertSeverity.LOW])
            }
        }

    def set_performance_threshold(self, metric_name: str, threshold_value: float) -> bool:
        """
        Set a performance threshold.

        Args:
            metric_name: Name of the metric to set threshold for
            threshold_value: Threshold value to set

        Returns:
            bool: True if threshold was set, False otherwise
        """
        if metric_name in self._performance_thresholds:
            old_value = self._performance_thresholds[metric_name]
            self._performance_thresholds[metric_name] = threshold_value
            self._logger.info(f"Threshold for {metric_name} updated from {old_value} to {threshold_value}")
            return True
        else:
            self._logger.warning(f"Unknown metric name for threshold: {metric_name}")
            return False

    def get_active_alerts(self) -> List[Dict[str, Any]]:
        """
        Get all active (unresolved) alerts.

        Returns:
            List[Dict[str, Any]]: List of active alerts
        """
        active_alerts = [alert for alert in self._alerts if not alert.resolved]
        return [
            {
                "id": alert.alert_id,
                "timestamp": alert.timestamp.isoformat(),
                "severity": alert.severity.value,
                "type": alert.alert_type.value,
                "message": alert.message,
                "details": alert.details
            }
            for alert in active_alerts
        ]

    def run_health_check(self) -> Dict[str, Any]:
        """
        Perform a comprehensive health check of the system.

        Returns:
            Dict[str, Any]: Health check results
        """
        # Collect current metrics
        current_metrics = self.collect_system_metrics()

        # Check thresholds and potentially generate new alerts
        new_alerts = self.check_thresholds(current_metrics)

        # Send notifications for new alerts
        for alert in new_alerts:
            self.send_alert_notification(alert)

        # Get system health status
        health_status = self.get_system_health_status()

        # Add additional health checks
        health_status["checks"] = {
            "service_responsive": True,  # Would check actual service responsiveness
            "database_connected": self._check_database_connection(),
            "external_apis_available": self._check_external_api_availability(),
            "disk_space_sufficient": current_metrics.disk_percent < 95.0,
            "memory_sufficient": current_metrics.memory_percent < 95.0
        }

        self._logger.info("Health check completed")
        return health_status

    def _check_database_connection(self) -> bool:
        """
        Check if the vector database is accessible.

        Returns:
            bool: True if database is accessible, False otherwise
        """
        try:
            # In a real implementation, this would test the actual database connection
            # For now, we'll return True as a placeholder
            # Example: test a simple operation like listing collections
            return True
        except Exception as e:
            self._logger.error(f"Database connection check failed: {str(e)}")
            return False

    def _check_external_api_availability(self) -> bool:
        """
        Check if external APIs (like Cohere) are available.

        Returns:
            bool: True if external APIs are available, False otherwise
        """
        try:
            # In a real implementation, this would test the actual API connections
            # For now, we'll return True as a placeholder
            # Example: make a simple API call to Cohere
            return True
        except Exception as e:
            self._logger.error(f"External API availability check failed: {str(e)}")
            return False


# Global instance of MonitoringService
monitoring_service = MonitoringService()


def get_monitoring_service() -> MonitoringService:
    """
    Get the global MonitoringService instance.

    Returns:
        MonitoringService: The global MonitoringService instance
    """
    return monitoring_service


# Background monitoring task that can be run periodically
def run_monitoring_cycle():
    """
    Run a single cycle of monitoring checks.

    This function would typically be called periodically by a scheduler or background task.
    """
    monitor = get_monitoring_service()

    # Collect metrics
    metrics = monitor.collect_system_metrics()

    # Check thresholds and generate alerts
    new_alerts = monitor.check_thresholds(metrics)

    # Send notifications for critical alerts
    for alert in new_alerts:
        if alert.severity in [AlertSeverity.HIGH, AlertSeverity.CRITICAL]:
            monitor.send_alert_notification(alert)

    return {
        "timestamp": metrics.timestamp,
        "metrics_collected": True,
        "new_alerts_generated": len(new_alerts)
    }


# Example of how to use the monitoring service in an API health endpoint
def get_health_status():
    """
    Get the current health status of the service.

    This would be called by a /health endpoint in the API.
    """
    monitor = get_monitoring_service()
    return monitor.get_system_health_status()


def get_detailed_metrics():
    """
    Get detailed metrics for monitoring dashboards.

    This would be called by a /metrics endpoint in the API.
    """
    monitor = get_monitoring_service()
    return monitor.get_metrics_summary(hours=1)  # Last hour of metrics