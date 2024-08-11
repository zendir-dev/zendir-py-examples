from nominalpy import Credentials, Simulation, Object, Component, types

from credential_helper import fetch_credentials

creds = fetch_credentials()
sim = Simulation(creds)

universe: Component = sim.get_system(types.SOLAR_SYSTEM)
universe.invoke("SetSphericalHarmonics", "earth", 2)