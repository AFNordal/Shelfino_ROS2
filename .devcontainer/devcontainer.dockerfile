FROM pla10/ros2_humble:amd64
ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /root

RUN apt clean
RUN apt update 
RUN apt upgrade -y
RUN apt install -y git wget

RUN apt install -y libcgal-dev libcgal-qt5-dev

# # python
ARG python=python3.10
RUN apt install -y ${python} python3-pip 
RUN apt install -y python3-rosdep

RUN rosdep init
RUN rosdep update

RUN echo "alias pip=pip3" >> ~/.bash_aliases
RUN echo "alias python=python3" >> ~/.bash_aliases

RUN pip3 install --upgrade pip setuptools
RUN pip3 install black
RUN pip3 install numpy matplotlib tornado
RUN pip3 install shapely
RUN pip3 install thefuck

ENV MPLBACKEND=WebAgg

# gitconfig
#COPY ssh-keys /root/.ssh

RUN git config --global core.fileMode false
RUN git config --global --add safe.directory "*"
RUN git config --global user.email "vibekekd@stud.ntnu.no"
RUN git config --global user.name "VibekeKD"

# remote display
RUN echo "export DISPLAY=host.docker.internal:0.0" >> /root/.bashrc
RUN echo "export LIBGL_ALWAYS_SOFTWARE=1" >> /root/.bashrc

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "eval \"$(thefuck --alias)\""  >> /root/.bashrc