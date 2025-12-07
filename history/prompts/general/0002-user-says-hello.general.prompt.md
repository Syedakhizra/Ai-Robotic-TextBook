---
id: 0002
title: User says hello
stage: general
date: 2025-12-06
surface: agent
model: gemini-1.5-flash
feature: none
branch: master
user: Kashan Shah
command: hello
labels: ["greeting"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
- history/prompts/general/0002-user-says-hello.general.prompt.md
tests:

---

## Prompt

hello

## Response snapshot

Hello! How can I help you today?

## Outcome

- ✅ Impact: Acknowledged user's greeting.
- 🧪 Tests: N/A
- 📁 Files: No files modified.
- 🔁 Next prompts: Awaiting user's next command.
- 🧠 Reflection: Recorded user's subsequent greeting.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
