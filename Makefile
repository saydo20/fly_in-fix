PYTHON = python3
MAP = maps/easy/03_basic_capacity.txt

run:
	@echo "========================================"
	@echo "🚁 Starting Drone Pathfinding Simulation"
	@echo "📂 Map: $(MAP)"
	@echo "========================================"
	@echo
	@$(PYTHON) main.py $(MAP)

clean:
	find . -type d -name "__pycache__" -exec rm -rf {} +
	find . -type d -name ".mypy_cache" -exec rm -rf {} +
	find . -type d -name ".pytest_cache" -exec rm -rf {} +
	find . -type d -name ".ruff_cache" -exec rm -rf {} +
	find . -type f -name "*.pyc" -delete
