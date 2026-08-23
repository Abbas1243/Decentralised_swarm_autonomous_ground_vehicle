# Shared Infrastructure Setup Guide

## Table of Contents
1. [GitHub Repository Setup](#github-repository-setup)
2. [Branching Strategy](#branching-strategy)
3. [Shared Simulation Environment](#shared-simulation-environment)
4. [Communication Protocol](#communication-protocol)
5. [Weekly Meeting Schedule](#weekly-meeting-schedule)

---

## GitHub Repository Setup

### Initial Repository Configuration

```bash
# Create new repository on GitHub, then clone locally
git clone https://github.com/your-org/project-name.git
cd project-name

# Initialize with standard structure
mkdir -p {src,tests,docs,config,scripts,environments}
touch README.md .gitignore LICENSE
```

### Repository Structure

```
project-name/
├── .github/
│   ├── workflows/          # CI/CD pipelines
│   ├── ISSUE_TEMPLATE/     # Issue templates
│   └── PULL_REQUEST_TEMPLATE.md
├── src/                    # Source code
│   ├── agents/            # Agent implementations
│   ├── simulation/        # Simulation core
│   └── utils/             # Shared utilities
├── tests/                 # Test suites
├── docs/                  # Documentation
├── config/                # Configuration files
├── environments/          # Environment specifications
├── scripts/               # Utility scripts
├── .gitignore
├── README.md
├── requirements.txt       # Python dependencies
└── environment.yml        # Conda environment spec
```

### Essential Files

**.gitignore**
```
# Python
__pycache__/
*.py[cod]
*$py.class
*.so
.Python
env/
venv/
ENV/
.venv

# IDEs
.vscode/
.idea/
*.swp
*.swo

# OS
.DS_Store
Thumbs.db

# Project specific
*.log
data/raw/*
!data/raw/.gitkeep
outputs/
checkpoints/
.env
```

**README.md Template**
```markdown
# Project Name

## Quick Start
1. Clone repository
2. Set up environment (see below)
3. Run tests: `pytest tests/`
4. Start simulation: `python src/main.py`

## Environment Setup
See [Environment Setup](#shared-simulation-environment)

## Contributing
See [Branching Strategy](#branching-strategy)
```

---

## Branching Strategy

### Git Flow Model

```
main (production-ready)
  ├── develop (integration branch)
  │   ├── feature/agent-navigation
  │   ├── feature/communication-protocol
  │   ├── feature/reward-system
  │   └── hotfix/simulation-crash
  └── release/v1.0.0
```

### Branch Types

| Branch Type | Naming Convention | Purpose | Base Branch | Merge To |
|------------|-------------------|---------|-------------|----------|
| `main` | `main` | Production-ready code | - | - |
| `develop` | `develop` | Integration branch | `main` | `main` |
| `feature/*` | `feature/description` | New features | `develop` | `develop` |
| `bugfix/*` | `bugfix/description` | Bug fixes | `develop` | `develop` |
| `hotfix/*` | `hotfix/description` | Urgent production fixes | `main` | `main` & `develop` |
| `release/*` | `release/v1.0.0` | Release preparation | `develop` | `main` & `develop` |

### Workflow Guidelines

**Creating a Feature Branch**
```bash
# Update develop
git checkout develop
git pull origin develop

# Create feature branch
git checkout -b feature/agent-pathfinding

# Work on feature
git add .
git commit -m "feat: implement A* pathfinding for agents"

# Push to remote
git push -u origin feature/agent-pathfinding
```

**Pull Request Process**
1. Create PR from feature branch to `develop`
2. Request review from at least 1 team member
3. Ensure CI/CD passes (tests, linting)
4. Address review comments
5. Squash and merge when approved

**Commit Message Convention**
```
<type>(<scope>): <subject>

Types:
- feat: New feature
- fix: Bug fix
- docs: Documentation only
- style: Formatting, no code change
- refactor: Code restructuring
- test: Adding tests
- chore: Maintenance tasks

Examples:
feat(agent): add collision detection
fix(simulation): resolve timestep synchronization issue
docs(api): update communication protocol specification
```

### Branch Protection Rules

**For `main` branch:**
- Require pull request reviews (minimum 2)
- Require status checks to pass
- Require branches to be up to date
- No force pushes
- No deletions

**For `develop` branch:**
- Require pull request reviews (minimum 1)
- Require status checks to pass
- No force pushes

---

## Shared Simulation Environment

### Environment Versioning

**Version Control Strategy**
- Lock all dependency versions
- Use semantic versioning for environment releases
- Document breaking changes
- Maintain changelog

### Python Environment Setup

**Using Conda (Recommended)**

Create `environment.yml`:
```yaml
name: simulation-env
channels:
  - conda-forge
  - defaults
dependencies:
  - python=3.10.12
  - numpy=1.24.3
  - pandas=2.0.3
  - matplotlib=3.7.2
  - scipy=1.11.1
  - pytest=7.4.0
  - black=23.7.0
  - flake8=6.0.0
  - pip=23.2.1
  - pip:
    - gym==0.26.2
    - stable-baselines3==2.0.0
    - tensorboard==2.13.0
```

Setup commands:
```bash
# Create environment
conda env create -f environment.yml

# Activate environment
conda activate simulation-env

# Update environment
conda env update -f environment.yml --prune

# Export your environment (for verification)
conda env export > environment-lock.yml
```

**Using pip + venv (Alternative)**

Create `requirements.txt`:
```
python==3.10.12
numpy==1.24.3
pandas==2.0.3
matplotlib==3.7.2
scipy==1.11.1
gym==0.26.2
stable-baselines3==2.0.0
tensorboard==2.13.0
pytest==7.4.0
black==23.7.0
flake8==6.0.0
```

Setup commands:
```bash
# Create virtual environment
python -m venv venv

# Activate (Linux/Mac)
source venv/bin/activate
# Activate (Windows)
venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Freeze exact versions
pip freeze > requirements-lock.txt
```

### Environment Verification Script

Create `scripts/verify_environment.py`:
```python
#!/usr/bin/env python3
"""Verify simulation environment setup"""

import sys
import pkg_resources

REQUIRED_PACKAGES = {
    'numpy': '1.24.3',
    'pandas': '2.0.3',
    'matplotlib': '3.7.2',
    'scipy': '1.11.1',
    'gym': '0.26.2',
}

def verify_environment():
    """Check if all required packages are installed with correct versions"""
    missing = []
    wrong_version = []
    
    for package, version in REQUIRED_PACKAGES.items():
        try:
            installed_version = pkg_resources.get_distribution(package).version
            if installed_version != version:
                wrong_version.append(f"{package}: expected {version}, got {installed_version}")
        except pkg_resources.DistributionNotFound:
            missing.append(f"{package}=={version}")
    
    if missing:
        print("❌ Missing packages:")
        for pkg in missing:
            print(f"  - {pkg}")
    
    if wrong_version:
        print("⚠️  Version mismatches:")
        for msg in wrong_version:
            print(f"  - {msg}")
    
    if not missing and not wrong_version:
        print("✅ Environment verification passed!")
        print(f"Python version: {sys.version}")
        return True
    
    return False

if __name__ == "__main__":
    success = verify_environment()
    sys.exit(0 if success else 1)
```

Run verification:
```bash
python scripts/verify_environment.py
```

### Docker Option (For Consistency)

Create `Dockerfile`:
```dockerfile
FROM python:3.10.12-slim

WORKDIR /app

# Install system dependencies
RUN apt-get update && apt-get install -y \
    git \
    && rm -rf /var/lib/apt/lists/*

# Copy environment files
COPY requirements.txt .

# Install Python dependencies
RUN pip install --no-cache-dir -r requirements.txt

# Copy project
COPY . .

# Verification
RUN python scripts/verify_environment.py

CMD ["python", "src/main.py"]
```

`docker-compose.yml`:
```yaml
version: '3.8'

services:
  simulation:
    build: .
    volumes:
      - ./src:/app/src
      - ./data:/app/data
      - ./outputs:/app/outputs
    environment:
      - PYTHONUNBUFFERED=1
    command: python src/main.py
```

---

## Communication Protocol

### Message Format Specification

**Standard Message Schema (JSON)**

```json
{
  "message_id": "uuid-v4",
  "timestamp": "2026-01-30T10:30:00.000Z",
  "sender_id": "agent_001",
  "recipient_id": "agent_002",
  "message_type": "status_update|request|response|broadcast",
  "priority": "low|normal|high|urgent",
  "payload": {
    "type": "specific_payload_type",
    "data": {}
  },
  "metadata": {
    "simulation_tick": 1500,
    "environment_state": "running"
  }
}
```

### Message Types

**1. Status Update**
```json
{
  "message_type": "status_update",
  "payload": {
    "type": "agent_state",
    "data": {
      "position": [10.5, 20.3],
      "velocity": [1.2, 0.8],
      "health": 100,
      "resources": {"energy": 85}
    }
  }
}
```

**2. Request**
```json
{
  "message_type": "request",
  "payload": {
    "type": "resource_request",
    "data": {
      "resource_type": "energy",
      "amount": 50,
      "urgency": "high"
    }
  }
}
```

**3. Response**
```json
{
  "message_type": "response",
  "payload": {
    "type": "resource_response",
    "data": {
      "request_id": "original-uuid",
      "status": "granted|denied|partial",
      "amount_provided": 30
    }
  }
}
```

**4. Broadcast**
```json
{
  "message_type": "broadcast",
  "recipient_id": "all",
  "payload": {
    "type": "alert",
    "data": {
      "alert_type": "danger",
      "location": [15.0, 25.0],
      "description": "Obstacle detected"
    }
  }
}
```

### Communication Frequencies

| Component | Update Frequency | Protocol |
|-----------|-----------------|----------|
| Agent State | 10 Hz (every 100ms) | Status Update |
| Sensor Data | 20 Hz (every 50ms) | Status Update |
| Inter-Agent Messages | On-demand | Request/Response |
| Environment Updates | 5 Hz (every 200ms) | Broadcast |
| Logging | 1 Hz (every 1s) | System Log |
| Telemetry | 0.1 Hz (every 10s) | Metrics Push |

### API Endpoints (if using REST)

```
GET    /api/v1/simulation/status       - Get simulation state
POST   /api/v1/simulation/start        - Start simulation
POST   /api/v1/simulation/stop         - Stop simulation
POST   /api/v1/simulation/reset        - Reset simulation

GET    /api/v1/agents                  - List all agents
GET    /api/v1/agents/:id              - Get agent details
POST   /api/v1/agents/:id/command      - Send command to agent

GET    /api/v1/messages                - Get message queue
POST   /api/v1/messages                - Post new message

GET    /api/v1/metrics                 - Get performance metrics
```

### Python Implementation Example

```python
from dataclasses import dataclass, asdict
from datetime import datetime
from typing import Any, Dict
import uuid
import json

@dataclass
class Message:
    sender_id: str
    recipient_id: str
    message_type: str
    payload: Dict[str, Any]
    message_id: str = None
    timestamp: str = None
    priority: str = "normal"
    metadata: Dict[str, Any] = None
    
    def __post_init__(self):
        if self.message_id is None:
            self.message_id = str(uuid.uuid4())
        if self.timestamp is None:
            self.timestamp = datetime.utcnow().isoformat() + 'Z'
        if self.metadata is None:
            self.metadata = {}
    
    def to_json(self) -> str:
        return json.dumps(asdict(self), indent=2)
    
    @classmethod
    def from_json(cls, json_str: str) -> 'Message':
        return cls(**json.loads(json_str))

# Usage
msg = Message(
    sender_id="agent_001",
    recipient_id="agent_002",
    message_type="request",
    payload={
        "type": "resource_request",
        "data": {"resource_type": "energy", "amount": 50}
    }
)

print(msg.to_json())
```

---

## Weekly Meeting Schedule

### Regular Meetings

| Meeting | Day | Time | Duration | Attendees | Purpose |
|---------|-----|------|----------|-----------|---------|
| **Team Standup** | Mon, Wed, Fri | 9:00 AM | 15 min | All | Quick sync, blockers |
| **Technical Review** | Tuesday | 2:00 PM | 60 min | All | Code review, architecture |
| **Sprint Planning** | Monday | 10:00 AM | 90 min | All | Week planning |
| **Integration Testing** | Thursday | 3:00 PM | 60 min | All | Test integration |
| **Retrospective** | Friday | 4:00 PM | 45 min | All | Week review |

### Meeting Templates

#### Daily Standup (15 min)

**Format:**
Each person shares:
1. What I completed since last standup (2 min)
2. What I'm working on today (1 min)
3. Any blockers or help needed (1 min)

**Tools:** Slack, Zoom, or in-person

**Notes:** Keep it brief, take detailed discussions offline

---

#### Sprint Planning (90 min)

**Agenda:**
1. Review previous week (15 min)
   - Completed tasks
   - Metrics/KPIs
   - Lessons learned

2. This week's goals (30 min)
   - Priority items
   - Dependencies
   - Risk assessment

3. Task assignment (30 min)
   - GitHub issue assignment
   - Estimation
   - Resource allocation

4. Integration checkpoints (15 min)
   - When to merge
   - Testing schedule
   - Review assignments

**Deliverables:**
- Updated GitHub project board
- Assigned issues
- Integration timeline

---

#### Technical Review (60 min)

**Agenda:**
1. Code reviews (20 min)
   - Open PRs
   - Design discussions
   - Best practices

2. Architecture updates (20 min)
   - System design changes
   - New components
   - Performance considerations

3. Technical debt (10 min)
   - Identified issues
   - Refactoring needs
   - Documentation gaps

4. Q&A / Knowledge sharing (10 min)

**Requirements:**
- PRs submitted 24h before meeting
- Design docs shared in advance

---

#### Integration Testing (60 min)

**Agenda:**
1. Environment verification (10 min)
   - Everyone on same version
   - Dependencies check
   - Configuration sync

2. Integration tests (30 min)
   - Run test suite
   - Cross-component testing
   - Performance benchmarks

3. Issue triage (15 min)
   - Bugs identified
   - Priority assignment
   - Quick fixes vs. backlog

4. Next steps (5 min)

**Success Criteria:**
- All tests pass
- No critical bugs
- Performance within targets

---

#### Retrospective (45 min)

**Format:** Start/Stop/Continue

1. What went well? (15 min)
2. What didn't go well? (15 min)
3. Action items for next week (15 min)

**Ground Rules:**
- Blameless culture
- Focus on process, not people
- Actionable outcomes

---

### Communication Channels

| Channel | Purpose | Response Time |
|---------|---------|---------------|
| **Slack #general** | General discussion | Best effort |
| **Slack #urgent** | Blocking issues | <30 min |
| **GitHub Issues** | Bug reports, features | <24 hours |
| **GitHub PRs** | Code review | <48 hours |
| **Email** | Formal communication | <24 hours |
| **Video Call** | Real-time collaboration | Scheduled |

### Meeting Artifacts

**All meetings should produce:**
- Meeting notes (shared doc)
- Action items with owners
- GitHub issues for tasks
- Updated project board

**Storage:**
- Meeting notes: `docs/meetings/YYYY-MM-DD-meeting-type.md`
- Decisions: `docs/decisions/ADR-XXX-title.md` (Architecture Decision Records)

---

## Getting Started Checklist

### Repository Setup
- [ ] Create GitHub repository
- [ ] Add all team members
- [ ] Set up branch protection
- [ ] Create initial directory structure
- [ ] Add .gitignore, README, LICENSE
- [ ] Configure GitHub Actions for CI/CD

### Environment Setup
- [ ] Create environment.yml or requirements.txt
- [ ] Test environment on each member's machine
- [ ] Run verification script
- [ ] Document any OS-specific issues
- [ ] Create Docker setup (optional)

### Communication Setup
- [ ] Define message schema
- [ ] Implement message classes
- [ ] Set up communication channels (Slack, etc.)
- [ ] Test message passing
- [ ] Document API endpoints

### Meeting Setup
- [ ] Schedule recurring meetings
- [ ] Create meeting templates
- [ ] Set up shared calendar
- [ ] Choose video conferencing tool
- [ ] Create docs/meetings directory

---

## Maintenance & Updates

### Weekly Tasks
- Update dependencies (check for security patches)
- Review and merge PRs
- Update documentation
- Run full test suite

### Monthly Tasks
- Environment version bump (if needed)
- Dependency audit
- Performance review
- Documentation review

### Version Updates
When updating the environment version:
1. Create new branch: `environment/vX.Y.Z`
2. Update environment.yml
3. Test thoroughly
4. Update CHANGELOG.md
5. Tag release
6. Notify team

---

## Troubleshooting

### Environment Issues
```bash
# Reset environment completely
conda deactivate
conda env remove -n simulation-env
conda env create -f environment.yml
conda activate simulation-env
python scripts/verify_environment.py
```

### Git Issues
```bash
# If branches diverged
git fetch origin
git rebase origin/develop

# If merge conflicts
git merge --abort  # start over
# or resolve conflicts manually
```

### Communication Issues
- Check message format against schema
- Verify network connectivity
- Check firewall settings
- Review logs for error messages

---

## Additional Resources

- [GitHub Flow Guide](https://guides.github.com/introduction/flow/)
- [Semantic Versioning](https://semver.org/)
- [Conventional Commits](https://www.conventionalcommits.org/)
- [Architecture Decision Records](https://adr.github.io/)

---

**Last Updated:** 2026-01-30  
**Version:** 1.0.0  
**Maintainer:** [Your Team Name]
