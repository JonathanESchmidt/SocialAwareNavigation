# SocialAwareNavigation

## Authors:
Alexandru Smau: **asmau21@student.aau.dk**
Christian Rønnest: **cwra18@student.aau.dk**
Jonathan Schmidt: **jschmi17@student.aau.dk**
Tristan Schwörer: **tschwo21@student.aau.dk**



## The Docker Integration of this repository is forked from https://github.com/nebohq/mac-ros

Install Docker on the Wanted platform (General Solution is installing Docker Desktop since this figures everything out for you) otherwise use apt or simillar
Clone the Repository to the wanted location
Run `docker-compose up --build` in the location of the repository

## Usage
1. Start terminal using the Docker Desktop CLI button or open using `docker-compose exec ros bash` in the correct Directory
2. from then on its regular ROS stuff like `catkin_make && source devel/setup.bash`
6. For ggraphical output open your browser to `localhost:8080/vnc.html` and click connect.


## Installing other packages
Edit the `Dockerfile` line that installs packages and rebuild the container using `docker-compose build`.
