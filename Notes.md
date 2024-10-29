# First meeting with Claude
Yolo v7/v8
Sony TV control
Kinect for windows
- Multiple?
    - Interfering?
    - Different frequencies
Voice control
- Already in yolo
Predictive control using cameras
- Machine Learning
Paddington campus lab?
Control devices with Raspberry Pis
ROS Bridge
Translate Kinect to Robot Maps
SLAM - Simultaneous Localisation and Mapping

Literature
- Smart Environment
- Robotics
- Person tracking
- 3D tracking
- Behaviour monitoring
- ROS
- Theory and principles behind it
- Not description

For thesis C
For aims in Thesis A
How can you evaluate your goals?
How did the literature evaluate themselves?

# Meeting with Claude regarding the presentation
Define the problem
Research smart home for context for the problem
Problem solving - not steps to get working
Explain the several steps of connection from Kinect to Ros2
Outline of plan


Thesis C how to verify success, how often things work?

Gesture recognition
Person tracking

Yolo Pose Models (Skeletons)

MVP + Stretch goals
Demonstrate by proxy


# Plan for Thesis B and C
Person tracking Gesture recognition Yolo Pose Models (Skeletons)
Controlling Devices
Web interface
- Control devices
- Set up new commands and gestures?


Smart home is about automation
About making simple mundane daily tasks easier,
freeing up time in your life for the things that matter more
I set out to do just that

# Things to research
Computer Vision
Computer Vision in home automation
- https://www.integrasources.com/blog/opencv-computer-vision-algorithms-iot-home-automation/
- https://ieeexplore.ieee.org/document/8697997

History of the Smart Home
- https://www.linkedin.com/pulse/how-smart-home-technology-has-evolved-from-beginning-today
- https://ubuntu.com/blog/the-evolution-of-the-smart-home-how-it-started-part-1
- https://www.afcdud.com/fr/smart-city/422-how-the-history-of-smart-homes.html
- Entranet YT (5 Vids) & Fabulous Home Automation (X10 and 1980s) https://www.youtube.com/watch?v=Xvz6_nvn0rU
    - The beginnings of a smart home
        - 1901 - Engine-powered vacuum cleaner on horse and drawn cart
        - 1903 - Electric iron
        - 1907 - Domestic vacuum cleaner
        - 1909 - One-sided electric toaster
        - 1913 - First refrigerator
        - 1919 - Automatic pop-up timer toaster
        - 1920s - home appliances promising consumers more time for leisure
        - 1927 - Iron with adjustable temperature control
        - 1927 - First garbage disposal
        - 1930s - Washing machine
    - The first device designed for home automation
        - 1966
            - ECHO IV - Electronic Computing Home Operator
            - Hand-crafted system with surplus electronic parts
            - Computerised many household chores
            - Engineered by Jim Sutherland
            - Large, 3000 Watts, required lots of technical know-how
            - Used to:
		- Compute shopping lists
		- Control temperature
		- Turn appliances on and off
		- Predict the weather
        - 1967 - The kitchen computer
            - Created by Honeywell
            - Offered in Neiman Marcus Christmas Catalogue
            - Computer made for the housewife
            - Used in the kitchen
            - Recipe storage device
            - 0.6 - 2.5MHz Processor
            - 4Kb of memory (expandable to 16 Kb)
            - No display
            - Amazingly beautiful and hopelessly impractical
            - $10,000
            - None ever sold
        - The beginnings of people imagining a future where homes would be interactive
        - First implementations
        - 1975
            - X10 Protocol
            - Developed in Scotland
            - Formed the basis for many domestic control installations for many decades
            - One of the first protocols to completely cover the home automation spectrum
                - Power
                - Lighting
                - Security
            - Capable of controlling 256 devices on one circuit
            - Sent messages through the property's existing AC electrical wiring
                - At the time this was an efficient way to send basic signals through large spaces
                - Controllers ranged from switches to programmable timers
            - Later advanced to be controllable from a computer
                - Schedule events
                - Decision-based sequences
    - 1980s
        - 3x Xanadu Houses
            - Solidify the concepts of home automation and computer control
            - Thorough implementations of lighting, heating and security
        - Home Robots
            - Became achievable
                - Microprocessors and baterries small enough to fit
            - Idea of robots as companions solidified through sci-fi movies
    - 1990s - Gerontechnology
        - Life Alert
            - Medical alert system designed to protect seniors
            - and family members in a home health emergency
        - 1991 - Consultancy office for assistive technology in the care sector
            - Started by mechanical engineer, Ad van Berlo
            - Beginning the design of healthcare technology
        - 1994 - Eindhoven UT started gerontechnology research
            - Home technology is one of the main sub-disciplines
            - Later called home automation
        - 1998 - Smart Homes (company)
            - Started by Corien van Berlo, Ad's partner
            - Aimed primarily to further the promotion of home automation
            - Execute demonstration projects
            - Start experiments
        - Early 1990s, people could not yet imagine what home automation would entail
        - Various projects took place in which older people could actually experience what home automation had to offer
    -  Early modern smart homes
        - 1984
            - American Association of House Builders coined the term "smart house"
                - Over the next decade, movies tried to depict it
        - 1990s
            - 1991 - Electrolux Trilobite
                - First robot vacuum
                - Heralded the integration of the first smart home products that were not hard-wired into the building
                - Truly smart appliances begun to emerge
            - Home computers became commonplace
            - Internet becoming more accessible and understandable
        - Early 2000s
            - More "smart" devices appeared
            - Idea of home appliances connecting to the internet popularised
                - Seen as the next big thing
        - June 2000
            - LG launched the first Internet refrigerator
            - Failed
            - Seen as unnecessary and expensive
            - >$20,000
        - 2002
            - Ambient Orb was named as one of the Ideas of the Year by Times Magazine
                - Monitored Dow Jones, personal portfolios, weather and more
                - Changed colours based on dynamic parameters
        - 2003 - 2004
            - Cooltown, Internet0, and The Disappearing Computer Initiative
                - seek to implement some of the ideas
            - Internet of Things term starts to appear
        - 2005
            - International Telecommunications Union published their first report on the topic
                - "A new dimension has been added to the world of information and communication technologies: from any time, anyplace connectivity for anyone, there is an additional dimension - connectivity for anything. Connections will multiply and create an entirely new dynamic network of networks – an Internet of Things."
            - Nabaztag, little WiFi rabbit
                - Could connect to each other
                - Spoke to you about stock market reports, news headlines, alarm clock alerts, etc.
        - Early 2000s
            - Different technologies began to emerge
            - Smart homes became a more affordable option
            - Domestic technologies, home networking, and other gadgets began to appear on store shelves
    - Smart Homes Today
        - Now a reality and not just for the eccentric and wealthy
        - Through the use of innovative technology, homeowners can turn their homes into state-of-the-art machines
            - That can be controlled and monitored from anywhere in the world
        - What is a smart home?
            - A home equipped with network-connected products via:
                - Wi-Fi,
                - Bluetooth, or
                - similar protocols for:
                    - controlling,
                    - automating, and
                    - optimising functions such as:
                        - temperature,
                        - lighting,
                        - security,
                        - safety, and
                        - entertainment
            - Any device in your home that uses electricity can be put on your home network
                - The home reacts at your command given by:
                    - Remote control
                    - Voice
                    - Smartphone or tablet
            - Provides its owners (at all times, regardless of whether anyone is home or not):
                - Comfort
                - Security
                - Energy efficiency
                - Convenience
    - THE NEXT STEP
        - AI!!!!!!!!!!!!!!!!!!

1999 Smart House film

Existing smart home software
- Home Assistant

Depth Camera for Home Automation
- https://medium.com/interaction-dynamics/how-to-use-machine-learning-for-home-automation-with-luxonis-depth-camera-uxtech-1-765418665b5

Problems with existing solutions 
- Motion detection for lighting
    - Any object moving causes lights to turn on, not just people
    - If you are watching TV and you move the lights turn on

Solutions
- With the Kinect camera and some AI 
    - We can detect if it is a person moving
    - We can detect if the person is still seated

- I want to implement a web interface as well
    - Reasoning:
        - I think it will provide more for the thesis
        - I am a strong believer that we should not replace existing functionality
            - But we should develop on TOP of it
        - We should not replace the ability to open blinds or turn lights on and off manually
            - We should add the option to be able to control them regularly
        - Technology is not fool-proof (YET) so we must cater to potential failures


# PROBLEMS WITH EXISTING SMART HOMES
https://reolink.com/blog/smart-home-frequent-issues/#4-automation-and-routine-failures
- Interoperability
    - Devices operating on different protocols cannot communicate
- Lack of true "intelligence" in a "smart" home
    - Smart devices are actually dumb
    - At best, they follow timers or simple algorithms
    - AI implementation makes smart devices truly smart
- Requirement for internet connection
    - Many devices require an internet connection to operate
    - Computation and automation running should all be done locally to solve this
- Automation and routine failure
    - Devices
- Privacy and Cyber security risks
    - Solved by having local computation


# Thesis C TODOs
- [x] Timing commands with gestures
- [x] Alternative interpretations of speech for commands
- [x] Response when command not recognised
- [x] Wake word
- [ ] Pattern matching for commands
- [x] User can upload their own commands

# Report
- Future plan

- Appendix:
    - user manual
    - how to use, etc.
    - extensive detail on setup and running
