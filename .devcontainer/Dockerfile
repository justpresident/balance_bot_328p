FROM debian:bookworm-slim

ARG TZ=UTC
ENV TZ="$TZ"

# Install development tools and AVR toolchain
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Basic tools
    ca-certificates \
    curl \
    wget \
    git \
    less \
    procps \
    sudo \
    fzf \
    zsh \
    man-db \
    unzip \
    jq \
    nano \
    vim \
    ripgrep \
    # AVR toolchain for atmega328p development
    gcc-avr \
    binutils-avr \
    avr-libc \
    avrdude \
    # Required for building ravedude (serial port support)
    libudev-dev \
    pkg-config \
    # Python for plotting scripts
    python3 \
    python3-matplotlib \
    python3-numpy \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Create developer user
ARG USERNAME=developer
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Add user to dialout group for serial port access
RUN usermod -aG dialout $USERNAME

# Persist bash history
RUN mkdir /commandhistory \
    && touch /commandhistory/.bash_history \
    && chown -R $USERNAME /commandhistory

# Set up workspace
RUN mkdir -p /workspace && chown -R $USERNAME:$USERNAME /workspace
WORKDIR /workspace

# Switch to developer user
USER $USERNAME

# Install Rust toolchain
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
ENV PATH=/home/${USERNAME}/.cargo/bin:${PATH}

# Install the specific nightly toolchain required for AVR development
RUN . "$HOME/.cargo/env" && \
    rustup toolchain install nightly-2024-03-22 --component rust-src && \
    cargo install ravedude

# Set up zsh with oh-my-zsh
RUN sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" "" --unattended \
    && echo 'export PROMPT_COMMAND="history -a" && export HISTFILE=/commandhistory/.bash_history' >> ~/.zshrc

# Set environment
ENV SHELL=/bin/zsh
ENV EDITOR=nano
