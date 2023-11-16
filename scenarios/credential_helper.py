'''
                    [ NOMINAL SYSTEMS ]
This code is developed by Nominal Systems to aid with communication 
to the public API. All code is under the the license provided along
with the 'nominalpy' module. Copyright Nominal Systems, 2023.

This method allows for easy exporting of custom-defined credentials,
specified within the parameters below. Importing this method into
scenario files enables easy access to the correct credentials of
the system. When using the public API, make sure to enter your access
key to the GLOBAL_KEY parameter.
'''

from nominalpy import Credentials

# Defines the credentials for a Local API connection
LOCAL_URL: str = "http://localhost"
LOCAL_PORT: int = 5001
LOCAL_KEY: str = ""

# Defines the credentials for a Public API connection
GLOBAL_URL: str = "https://api.nominalsys.com"
GLOBAL_PORT: int = None
GLOBAL_KEY: str = ""        # This is where the access key for the API should be added

# The flag that sets whether to use the public API or the local one.
USE_PUBLIC_API: bool = True


def fetch_credentials () -> Credentials:
    '''
    Constructs some sample credentials that can be used for 
    all files that are passing in credentials to the simulation.
    This works for both the public and local API, provided that
    the flag USE_PUBLIC_API is called.
    '''
    if USE_PUBLIC_API:
        return Credentials(GLOBAL_URL, GLOBAL_PORT, GLOBAL_KEY)
    return Credentials(LOCAL_URL, LOCAL_PORT, LOCAL_KEY)