# Releasing

A release is a version tag such as `v1.2.3` on a commit of `main`. Pushing the tag builds the
Windows and macOS bundles, tests them, tags the image that CI already tested for that commit,
and drafts a GitHub Release. Nothing is public until a software lead publishes the draft.

What people download:

| File | For | What it is |
| --- | --- | --- |
| `waybionic-ground-station-windows-<version>.zip` | Windows 11 | The launcher scripts, an embedded Python with pyserial, and a compose file that pulls the tested image by digest. Docker Desktop is the only prerequisite. |
| `waybionic-ground-station-macos-<version>.zip` | macOS (Apple Silicon) | The source at the tag with `Install WayBionic.command`. The Mac app runs ROS natively, so the first start builds the workspace on the Mac. |
| `SHA256SUMS` | both | The checksums of the two zips. |
| `ghcr.io/waybionic/waybionic_ground_station:v<version>` | Linux, or anyone using `docker compose` | The image the bundles run. |

Linux has no bundle: use the image with `docker compose` as in [BuildInstructions.md](../BuildInstructions.md).
Windows on ARM and Intel Macs are not built or tested.

## One-time setup (a GitHub organisation owner)

1. **Make the image public**, so a Windows laptop can pull it without logging in. Organisation
   > Packages > `waybionic_ground_station` > Package settings > Danger Zone > Change
   visibility > Public. Check it from a machine that is not logged in:
   `docker pull ghcr.io/waybionic/waybionic_ground_station:jazzy`. While it is private, the
   launcher tells people to run `docker login ghcr.io` with a token that has `read:packages`.
2. **Restrict who can start a release.** Anyone with write access can push a tag, and a tag
   starts the workflow. The workflow only releases a commit on `main` that has a tested image
   and only drafts a release, so a stray tag does little harm, but restrict it anyway:
   Settings > Rules > Rulesets > New tag ruleset, target `v*`, restrict creations, updates and
   deletions, with the software leads on the bypass list.
3. Optional: an Actions environment with required reviewers, if you want a second person to
   approve each release.

## Cutting a release

1. Merge what belongs in it. The CI run on `main` for the commit you release must have
   finished and passed, because it publishes the image the release is made from.
2. **Dry run** (recommended before the first release, and after changing anything in
   `packaging/`): Actions > Release > Run workflow, with a version like `0.0.0-test`. It
   builds both bundles, tests them on Windows and macOS runners and keeps them as an
   artifact. Pull requests that touch the packaging run the same thing. Nothing is
   published.
3. Tag the commit and push the tag:

   ```console
   git switch main && git pull
   git tag -a v1.2.3 -m "WayBionic Ground Station 1.2.3"
   git push origin v1.2.3
   ```

   Use a suffix such as `v1.2.3-rc1` for a release candidate; it is marked as a pre-release.
4. Watch the Release workflow. It stops early, with a message, if the commit is not on `main`
   or has no tested image yet; in the second case wait for CI and re-run the workflow.
5. Open the draft on the Releases page and read the notes. Before publishing a release that
   people will use on the arm, tick through this:
   - [ ] The Windows zip was unpacked on a Windows 11 laptop with Docker Desktop:
     `Check WayBionic Setup.cmd` passes, and `Start WayBionic Arm.cmd` reaches the Arm setup
     panel with the carrier plugged in.
   - [ ] On a Mac, `Install WayBionic.command` installs the app and the app opens.
   - [ ] The notes say if the arm's behaviour changed, for example its joint limits (see
     [handover.md](handover.md)).
6. Publish the draft.

## What the workflow guarantees

- **Only reviewed, tested code is released.** The tag must be on a commit that is an ancestor
  of `main`, so it went through a pull request, and that commit must already have an image
  from the CI run on `main`.
- **The image is not rebuilt.** The release adds a version tag to the image CI tested, by digest,
  and the Windows bundle names that digest, so what runs on the laptop is what the tests ran.
  A version tag that already exists with another digest is an error: versions never move.
- **The Windows bundle is checked on Windows.** The runner unpacks it, parses the launcher in
  Windows PowerShell 5.1, checks that the bundled `python.exe` is signed by the Python
  Software Foundation, imports pyserial, and runs the launcher's `-SelfTest`. The macOS runner
  checks the installer, the app and the version stamp. The compose file is validated with
  `docker compose config`, pinned by digest and with nothing to build.
- **Downloads can be checked.** `SHA256SUMS` lists the zips, and every file has a build
  attestation: `gh attestation verify <file> --repo Waybionic/waybionic_ground_station`.
- **The zips are reproducible.** The same inputs give the same bytes (`SOURCE_DATE_EPOCH` is
  the commit time), so a rebuild can be compared with a release.
- Actions are pinned by commit and the workflow has the least permission each job needs.

## If something goes wrong

- **The workflow failed before the draft:** fix the cause and re-run the failed jobs. If the tag
  itself was wrong, delete it (`git push origin :refs/tags/v1.2.3`), fix, and tag again.
- **A bad release was published:** delete or unpublish the release and cut a new patch
  version. The image tag stays, because versions never move; saying so in the new notes is
  enough.
- **A Windows user gets "unauthorized":** the image is still private; see the one-time setup.

## Updating what the bundles pin

- **Python** (`packaging/windows/python-embed.json`): pick the new embeddable zip from
  python.org, download it with its `.sigstore` file, and verify it before pinning its SHA-256:

  ```console
  python -m pip install sigstore
  python -m sigstore verify identity --bundle <zip>.sigstore \
    --cert-identity <release manager> --cert-oidc-issuer https://github.com/login/oauth <zip>
  ```

  The release manager's identity for each Python series is listed at
  <https://www.python.org/downloads/metadata/sigstore/> (3.14 is `hugo@python.org`). Then
  open a pull request; its dry run tests the new Python on Windows.
- **pyserial** (`packaging/windows/requirements.txt`): update the version and the SHA-256 of
  the wheel from PyPI.
- **Actions** (`.github/workflows/release.yml`): replace the commit SHAs by hand, taking each
  from the release you want (`gh api repos/actions/checkout/git/ref/tags/<tag>`).
