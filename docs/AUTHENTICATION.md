# Authentication and credential handling

## Project boundary

This repository contains bare-metal embedded firmware. It has no network
server, user database, login page, password check, OAuth exchange, JWT,
cookie/session storage, or API-token loader. Hardware input and SWD programming
are device-control interfaces, not user authentication.

Names such as `DAUTHCTRL` and `DAUTHSTATUS` in ARM/CMSIS headers refer to MCU
debug-access control. They are unrelated to Web authentication and are not an
application credential flow. The third-party SDK containing those headers is
not distributed in this repository.

## Runtime request flow

```text
GPIO/timer interrupt → driver state → control loop → motor/OLED/Flash output
```

No step sends credentials or tokens over a network. W25Q64 stores robot tuning
parameters with versioning and CRC; it does not store passwords or access
tokens.

## GitHub publishing flow

Publishing is operationally separate from the firmware:

```text
repository owner
  → authorizes a GitHub App for selected repositories
  → Codex invokes the connected GitHub capability
  → GitHub checks installation and repository permissions
  → Git objects and the main branch are updated
```

The publishing process does not ask for or place a GitHub password, personal
access token, OAuth token, or private key in this repository. Connector token
storage, refresh, and expiry are managed outside the project by the connection
provider and are not exposed to the firmware. Access can be narrowed or revoked
from the connected-app and GitHub installation settings.

## Repository rules

- Never commit `.env` files, access tokens, private keys, or device secrets.
- Run a credential scan before every public release.
- Keep the separately licensed TI SDK under ignored `external/` storage.
- If future network features are added, document their trust boundary and use
  environment- or platform-managed secrets rather than source literals.
