"""
                    [ ZENDIR ]
This code is developed by Zendir to aid with communication
to the public API. All code is under the the license provided along
with the 'zendir' module. Copyright Zendir, 2025.

This method allows for easy exporting of custom-defined credentials,
specified within the parameters below. Importing this method into
scenario files enables easy access to the correct credentials of
the system. When using the public API, make sure to enter your access
key to the GLOBAL_KEY parameter.
"""

from zendir import Client

# Defines the credentials for a Local API connection
LOCAL_PORT: int = 25565

# Defines the credentials for a Public API connection
GLOBAL_URL: str = "https://api.zendir.io"
API_TOKEN: str = ""  # This is where the access key for the API should be added

# The flag that sets whether to use the public API or the local one.
USE_PUBLIC_API: bool = True


def fetch_client() -> Client:
    """
    Constructs some sample credentials that can be used for
    all files that are passing in credentials to the simulation.
    This works for both the public and local API, provided that
    the flag USE_PUBLIC_API is called.

    :returns:   The credentials for the API connection
    :rtype:     Credentials
    """

    if USE_PUBLIC_API:
        return Client(url=GLOBAL_URL, token=API_TOKEN)
    return Client.create_local(LOCAL_PORT)
