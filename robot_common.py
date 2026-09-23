
# ---------------- Intent message signing (HMAC-SHA256) ----------------
# Pre-shared-key MVP: every robot is launched with the same hmac_key
# parameter. This stops the spoofed-intent attack demonstrated by
# verify_log_not_process.py (an unsigned fake peer at priority 1.0
# could previously freeze the whole fleet). Roadmap: SROS2 / DDS-Security
# with per-robot X.509 identities and real key rotation -- a PSK is an
# honest MVP, not a claim of full PKI.

import hmac
import hashlib
import json


def _canonicalize(payload: dict) -> bytes:
    return json.dumps(payload, sort_keys=True, separators=(',', ':')).encode('utf-8')


def sign_payload(payload: dict, key: str) -> str:
    return hmac.new(key.encode('utf-8'), _canonicalize(payload), hashlib.sha256).hexdigest()


def verify_payload(payload: dict, signature: str, key: str) -> bool:
    if not signature:
        return False
    return hmac.compare_digest(sign_payload(payload, key), signature)
