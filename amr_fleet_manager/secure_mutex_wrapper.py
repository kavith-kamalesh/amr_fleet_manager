import hmac
import hashlib
import json
import os
from std_msgs.msg import String

FLEET_SECRET = os.environ.get("AMR_FLEET_SECRET", "DEV_ONLY_INSECURE_DEFAULT_KEY").encode()


def sign_and_publish_mutex(publisher, state: str):
    signature = hmac.new(FLEET_SECRET, state.encode(), hashlib.sha256).hexdigest()
    payload = json.dumps({"state": state, "signature": signature})
    publisher.publish(String(data=payload))


def verify_mutex_signature(state: str, signature: str) -> bool:
    expected = hmac.new(FLEET_SECRET, state.encode(), hashlib.sha256).hexdigest()
    return hmac.compare_digest(expected, signature)
