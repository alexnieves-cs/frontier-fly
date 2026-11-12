# Releasing

Four things ship from this repository. Each is built and tested in CI on every change; the site and the
container image publish on their own.

| What | Where | Workflow | Publishes |
| --- | --- | --- | --- |
| Site | GitHub Pages | `pages.yml` | On every push to main |
| Container image (CLI) | `ghcr.io/alexnieves-cs/frontier-fly` | `docker.yml` | On push to main (`latest`, `sha-<commit>`) and on `v*` tags (`1.2.3`, `1.2`) |
| Python package `frontierfly` | PyPI (not published yet) | `pypi.yml` | On a published GitHub release, or by hand with **publish** set, once the `PYPI_PUBLISHING` variable is `true` |
| Policy weights and model card | Hugging Face Hub | `huggingface.yml` | Only when run by hand with **upload** checked |

A GitHub release also triggers `release.yml`, which attaches the tech report PDF, and Zenodo archives the
release under the concept DOI [10.5281/zenodo.23717793](https://doi.org/10.5281/zenodo.23717793).

## Container image

Nothing to set up: the workflow pushes with the repository's own token. The first push creates the package
under the owner's account, private. To let anyone `docker pull` it, open the package page
(github.com/users/alexnieves-cs/packages/container/package/frontier-fly), **Package settings**, and set the
visibility to public; check that it is linked to this repository. (Done for this repository: the image pulls anonymously.)

## Python package on PyPI

The wheel is pure Python plus the bundled simulator (`npm run build:py`), so one `py3-none-any` wheel
covers every platform; it needs Node 22+ on `PATH` at run time.

`frontierfly` is not on PyPI yet: until the trusted publisher below exists, install it from a checkout
(see [python/README.md](../python/README.md#install)). The workflow builds and tests the wheel either way, and
skips the upload with a notice, not a failure, while publishing is not configured.

Once, on [pypi.org](https://pypi.org/manage/account/publishing/) (and on
[test.pypi.org](https://test.pypi.org/manage/account/publishing/) to rehearse), add a pending trusted
publisher: project `frontierfly`, owner `alexnieves-cs`, repository `frontier-fly`, workflow `pypi.yml`,
environment `pypi` (`testpypi` on TestPyPI). No token goes into the repository. Then set the repository
variable `PYPI_PUBLISHING` to `true` (Settings, Secrets and variables, Actions, Variables).

Each release:

1. Bump the version in `package.json`; `python/pyproject.toml` and `frontierfly.__version__` follow it (a
   test checks), and PyPI never accepts the same version twice.
2. Optionally rehearse: Actions, **pypi**, **Run workflow** on main, publish `testpypi`; check
   `pip install -i https://test.pypi.org/simple/ --extra-index-url https://pypi.org/simple frontierfly`.
3. Publish the GitHub release (`v1.2.3`). `pypi.yml` uploads the wheel to PyPI, `release.yml` attaches the
   report PDF and `docker.yml` tags the image. Running **pypi** by hand with publish `pypi` does the same
   upload.

## Policy on the Hugging Face Hub

`npm run hf:stage` writes `dist-hf/`: `policy.bin`, `pretrained.bin`, the result JSON, and a model card
generated from `docs/headline.json` and `bench/results/`, so the card's numbers are the README's. Running the
workflow without **upload** stages the same folder as an artifact and prints the card on the run page, to
review first.

Once, on [huggingface.co](https://huggingface.co/settings/tokens): create a token with write access and save it
as the repository secret `HF_TOKEN` (Settings, Secrets and variables, Actions). Then Actions,
**huggingface**, **Run workflow**, with the model repository (`<user>/frontier-fly-policy`) and **upload**
checked. The workflow creates the repository if it does not exist (public) and uploads a new commit to it.

Republish whenever the shipped checkpoint or the headline numbers change.
