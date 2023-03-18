PASSWD_PATH="$HOME/.vnc/passwd"

if ! [ -f $PASSWD_PATH ]; then
    # Set the password
    echo "$PASSWORD" | vncpasswd -f >> $PASSWD_PATH && chmod 600 $PASSWD_PATH
    # Apply permissions
    sudo chown root:root -R $HOME/
    sudo find $HOME/ -name '*.desktop' -exec chmod $verbose a+x {} +
fi

alias startvnc='vncserver :1 -nohttpd -depth 32 -geometry 1024x640 -name "Lunabase"'
alias rviz='ros2 run rviz2 rviz2'
