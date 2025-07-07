"""
                    [ NOMINAL SYSTEMS ]
This code is developed by Nominal Systems to aid with communication
to the public API. All code is under the the license provided along
with the 'nominalpy' module. Copyright Nominal Systems, 2024.

This method allows for easy exporting of custom-defined credentials,
specified within the parameters below. Importing this method into
scenario files enables easy access to the correct credentials of
the system.
"""

from nominalpy import Client

# Defines the credentials
LOCAL_URL: str = "http://127.0.0.1:25565"
GLOBAL_URL: str = "https://api.zendir.io"
GLOBAL_KEY: str = ""

# The flag that sets whether to use the public API or the local one.
USE_PUBLIC_API: bool = False


def fetch_client() -> Client:
    """
    Construct a sample client that can be used for all files that are
    passing in credentials to the simulation. This works for both the
    public and local API, provided that the flag USE_PUBLIC_API is called.
    """
    if USE_PUBLIC_API:
        return Client(GLOBAL_URL, token=GLOBAL_KEY)
    return Client(LOCAL_URL)
