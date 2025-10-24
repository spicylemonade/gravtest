PY := python3
VENV := .venv
PIP := $(VENV)/bin/pip
PYTHON := $(VENV)/bin/python
PYTEST := $(VENV)/bin/pytest

.PHONY: help
help:
	@echo Targets:
	@echo '  make venv        - Create virtualenv'
	@echo '  make install     - Install requirements'
	@echo '  make test        - Run unit tests'
	@echo '  make run         - Run headless demo'
	@echo '  make gui         - Run pygame demo (requires pygame)'
	@echo '  make paper       - Build LaTeX paper (requires pdflatex)'

$(VENV):
	$(PY) -m venv $(VENV)

venv: $(VENV)

install: venv
	$(PIP) install -r requirements.txt
	@echo 'Optional: install pygame via: $(PIP) install pygame'

test: install
	$(PYTEST) -q

run: install
	$(PYTHON) run.py --headless --scenario circular-orbit --steps 2000 --dt 0.005

gui: install
	@echo 'Attempting to run pygame demo; ensure pygame is installed.'
	$(PYTHON) run.py --scenario circular-orbit --steps 100000 --dt 0.005

paper:
	@if [ -f docs/paper.tex ]; then \
	  cd docs && pdflatex -interaction=nonstopmode paper.tex || true; \
	  cd docs && pdflatex -interaction=nonstopmode paper.tex || true; \
	else \
	  echo 'docs/paper.tex not found. Create it to build the paper.'; \
	fi
