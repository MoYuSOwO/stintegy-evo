"""How fast OpenF1 answers from this machine: one fresh half hour of one car."""
import random, time, urllib.request

driver = random.choice([4, 10, 11, 14, 18, 22, 23, 24, 27, 31, 55, 77, 81])
minute = random.randint(20, 59)
url = ("https://api.openf1.org/v1/location?session_key=9558"
       f"&driver_number={driver}&date>2024-07-07T14:{minute:02d}:00&date<2024-07-07T15:{minute:02d}:00")
start = time.perf_counter()
with urllib.request.urlopen(url, timeout=300) as r:
    body = r.read()
took = time.perf_counter() - start
print(f"{len(body) / 1e6:.2f} MB in {took:.1f} s = {len(body) / 1e3 / took:.0f} KB/s")
