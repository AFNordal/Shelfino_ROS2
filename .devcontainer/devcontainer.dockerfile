FROM pla10/ros2_humble:amd64
ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /root

RUN apt clean
RUN apt update 
RUN apt upgrade -y
RUN apt install -y git 

# # c++ libraries
# WORKDIR /include

# # python
ARG python=python3.11
RUN apt update && apt install -y ${python} python3-pip 
RUN apt install -y python3-rosdep

ARG WORKSPACE_FOLDER_ARG
# RUN echo ${WORKSPACE_FOLDER_ARG}
# WORKDIR ${WORKSPACE_FOLDER_ARG}
# RUN bingbong ${WORKSPACE_FOLDER_ARG}

RUN rosdep init
RUN rosdep update
# RUN rosdep install --from-paths ${WORKSPACE_FOLDER_ARG}/src --ignore-src -r -y && pwd

# python3-tk
# # RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/${python} 1
# # RUN update-alternatives --config python3

RUN printf "%s\n" "alias pip=pip3\n" "alias python=python3" > ~/.bash_aliases

RUN pip3 install --upgrade pip setuptools
RUN pip3 install black
RUN pip3 install numpy matplotlib tornado
RUN pip3 install shapely

ENV MPLBACKEND=WebAgg

# gitconfig
COPY ssh-keys /root/.ssh

RUN git config --global core.fileMode false
# RUN git config --global core.autocrlf true
RUN git config --global --add safe.directory "*"
RUN git config --global user.email "nordalrasmus01@gmail.com"
RUN git config --global user.name "Rasmus Anker Fossen Nordal"

# remote display
WORKDIR /root

RUN echo "export DISPLAY=host.docker.internal:0.0" >> .bashrc
RUN echo "export LIBGL_ALWAYS_SOFTWARE=1" >> .bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> .bashrc