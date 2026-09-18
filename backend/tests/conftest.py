import os

# Тесты не ходят в сеть за рельефом: используется только кэш data/dem/.
os.environ.setdefault("DEM_OFFLINE", "1")
