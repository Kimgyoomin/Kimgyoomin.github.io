# Gyoomin Kim — Robotics Portfolio

Personal portfolio for robotics and autonomous systems work.

## Portfolio V1

The first version focuses on a concise technical story rather than a generic developer profile:

- ROS 2 Humble / Nav2 autonomous navigation
- FAST-LIO and GenZ-ICP localization integration
- FastDEM 2.5-D elevation mapping
- Heightmap Wavefront + risk-aware A* / RRT* terrain planning
- LCM bridge from `/cmd_vel` to a low-level learned locomotion controller
- Legged robot / manufacturing-environment research context

The website is intentionally a lightweight static site so it can be hosted directly with GitHub Pages without paid infrastructure.

## Local preview

```bash
python3 -m http.server 8000
```

Then open `http://localhost:8000`.

## Deployment

Merge `feature/portfolio-v1` into `main`, then enable GitHub Pages for the `main` branch if it is not already enabled.

Expected public URL:

`https://kimgyoomin.github.io/`

## Project source links

This portfolio repository is presentation-only. Research/project repositories are referenced by links and are not modified as part of portfolio development.
