from typing import NamedTuple

class RevConfig(NamedTuple):
    """Configuration settings for REV hardware."""
    fw_set_retry_delay_sec: float = 0.01 # How long to wait between retries if hardware returns an error during configuration
    fw_set_retries: int = 10 # How many times to retry if hardware returns an error during configuration
