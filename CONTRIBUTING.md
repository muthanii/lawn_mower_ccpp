# Contributing

Thank you for your help. This guide tells you how to propose a change.

## Report a problem

Open an issue. Include:

- Your Windows and WSL versions (`wsl --version`).
- Your GPU model.
- The steps that cause the problem.
- The full error output.

## Make a change

1. Fork the repository.
2. Make a branch from `main`:

   ```bash
   git checkout -b fix/gui-not-opening
   ```

3. Make your change.
4. Rebuild the container and make sure the steps in the [README](README.md) still work.
5. Commit with a [Conventional Commits](https://www.conventionalcommits.org/) message.
6. Push the branch and open a pull request against `main`.

## Commit messages

The release process reads commit messages to set the next version and write the changelog. Because of this, each commit message and pull request title must use this format:

```text
<type>(<optional scope>): <description>
```

Use one of these types:

| Type | Use for | Version change |
| --- | --- | --- |
| `feat` | A new feature | Minor |
| `fix` | A bug fix | Patch |
| `docs` | Documentation only | None |
| `ci` | CI and release workflows | None |
| `chore` | Maintenance | None |
| `refactor` | A code change that does not change behavior | None |

For a breaking change, add `!` after the type, for example `feat!: drop WSL1 support`.

Examples:

```text
feat(devcontainer): add RViz to the image
fix: source the workspace overlay in new shells
docs: explain how to use a non-NVIDIA GPU
```

A CI check rejects pull request titles that do not use this format. Pull requests are squash merged, so the title becomes the commit message on `main`.

## Checks

CI runs on each pull request. It does these jobs:

- Builds the dev container image.
- Builds and tests all packages in `src/` with `colcon`.
- Checks the pull request title.

All checks must pass before a merge.

## Releases

You do not make releases by hand. [Release Please](https://github.com/googleapis/release-please) does these steps:

1. After each merge to `main`, it opens or updates a release pull request. This pull request updates `CHANGELOG.md` and the version.
2. When a maintainer merges the release pull request, it makes a Git tag and a GitHub release.

The Release Please settings are in `.github/release-please/`.
