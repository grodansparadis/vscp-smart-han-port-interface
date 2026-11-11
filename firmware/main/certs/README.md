# Certificates Directory Convention

Certificate and key file convention for this directory:

- Tracked example or demo material should use the suffix `.example.pem`.
- Real local private keys should use either `prvtkey.pem` or `*.local.pem` and must stay untracked.
- Public demo certificates may be tracked when they are intentionally non-sensitive.

Recommended usage:

1. Keep tracked examples clearly marked, such as `prvtkey.example.pem`.
2. Copy an example to a local filename before use.
3. Never commit real private key contents to this repository.

Examples:

- `prvtkey.example.pem`: tracked placeholder or demo key material.
- `prvtkey.pem`: local private key, ignored by git.
- `device-cert.local.pem`: local certificate or key material, ignored by git.

The focused Codacy and Trivy setup assumes that tracked files in this directory are examples or public material, while real private keys remain local only.
