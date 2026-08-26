# Zendir Python Examples

[![PyPI Version](https://img.shields.io/pypi/v/zendir.svg)](https://pypi.org/project/zendir/)

This repository includes a number of example scenario files that can be used as the basis for interacting with the Zendir API. Each of the scenarios showcases a particular use-case of the API. In order to use these examples, ensure that `zendir` is installed from Python pip. The `master` branch of this repository will align with the latest version of the `zendir` Python mpodule.

The sun pointing scenario uses reaction wheels and flight software to orient a spacecraft's solar panel towards the sun, charging the battery when not in the Earth's eclipse.

---

### API Tokens

The Zendir API requires an active API token for accessing the simulation. API tokens can be obtained from the official Zendir website, at https://zendir.io. Please make sure your token is active and has available credits.

---

### Credential Helper

Example scenarios under `scenarios/<category>/` (for example `dynamics`, `orbits`, `power`, `sensors`, `telemetry`, and `thermal`) and the RPO TestBed plays use `credential_helper.py` at the **repository root** to load credentials (including the API access token). That script creates a `zendir.Client` object for simulation access. Before running scenario files, fill in the `API_TOKEN` parameter in `credential_helper.py` with a valid API token.

Notebook tutorials live under `scenarios/tutorials/`.
