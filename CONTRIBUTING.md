# Commits and releases

## Make a change

1. Make a branch from `main`.
2. Commit the change and push the branch.
3. Open a pull request against `main`.
4. When all checks pass, squash merge the pull request.

The pull request title becomes the commit message on `main`.

## Commit messages

Release Please reads the commit messages to set the next version and write the changelog. Each commit message and pull request title must use this format:

```text
<type>(<optional scope>): <description>
```

| Type | Use for | Version change |
| --- | --- | --- |
| `feat` | A new feature | Minor |
| `fix` | A bug fix | Patch |
| `docs` | Documentation only | None |
| `ci` | CI and release workflows | None |
| `chore` | Maintenance | None |
| `refactor` | A code change that does not change behavior | None |

For a breaking change, add `!` after the type, for example `feat!: change the robot model`.

## Checks

CI runs on each pull request:

- Builds the dev container image.
- Builds and tests all packages in `src/` with `colcon`.
- Checks that the pull request title uses the commit message format.

## Releases

1. After each merge to `main`, Release Please opens or updates a release pull request. This pull request updates `CHANGELOG.md` and the version.
2. When you merge the release pull request, Release Please makes a `vX.Y.Z` tag and a GitHub release.

The Release Please settings are in `.github/release-please/`.
