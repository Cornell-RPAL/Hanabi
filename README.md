# Hanabot
**Hanabot** is a robot designed to play Hanabi with a human teammate. They are embodied with Baxter, and use algorithms such as computer vision and NLP to interact with humans. They ~~are~~ will be accessible through a terminal interface for a simple game play.

## What is Hanabi?
[Hanabi](https://en.wikipedia.org/wiki/Hanabi_(card_game)) is a cooperative card game that relies on implicit communication among players. 

## Installation
Hanabot needs to run on an Ubuntu machine that is connected with a Baxter unit. Instructions for setting up Baxter can be found [here](http://sdk.rethinkrobotics.com/wiki/Baxter_Setup). 
- Requirements:  `python 3.7` and `pipenv`
- In the directory of Hanabot, run `pipenv install`
- To update, run `git pull`, then `pipenv sync`

## Quick Start
Command Line:
`$ python main.py`
