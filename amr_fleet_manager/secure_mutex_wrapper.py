import hmac
import hashlib
import json
import os
import time
from std_msgs.msg import String

FLEET_SECRET = os.environ.get("AMR_FLEET_SECRET", "DEV_ONLY_INSECURE_DEFAULT_KEY").encode()
MAX_CLEARANCE_AGE_SEC = 1.0


def _mac(state: str, ts: float) -> str:
    return hmac.new(FLEET_SECRET, f"{state}|{ts:.3f}".encode(), hashlib.sha256).hexdigest()


def sign_and_publish_mutex(publisher, state: str):
    ts = round(time.time(), 3)
    publisher.publish(String(data=json.dumps(
        {"state": state, "ts": ts, "signature": _mac(state, ts)})))


def verify_mutex_signature(state: str, signature: str, ts=None,
                           max_age: float = MAX_CLEARANCE_AGE_SEC) -> bool:
    """Valid only if the MAC matches AND the timestamp is fresh (blocks replay)."""
    try:
        ts = float(ts)
    except (TypeError, ValueError):
        return False
    if abs(time.time() - ts) > max_age:
        return False
    return hmac.compare_digest(_mac(state, ts), str(signature))
