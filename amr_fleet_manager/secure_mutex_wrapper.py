import hmac
import hashlib
import json
from std_msgs.msg import String

FLEET_SECRET = b"BEL_DEFENCE_SIH26123_SECURE_KEY"

def sign_and_publish_mutex(publisher, state: str):
    """
    Wraps the standard ROS2 publisher to inject an HMAC-SHA256 signature.
    Usage: replace publisher.publish(String(data=state)) 
           with sign_and_publish_mutex(publisher, state)
    """
    signature = hmac.new(FLEET_SECRET, state.encode(), hashlib.sha256).hexdigest()
    payload = json.dumps({
        "state": state,
        "signature": signature
    })
    publisher.publish(String(data=payload))
