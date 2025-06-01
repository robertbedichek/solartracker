#!/bin/zsh

sudo launchctl bootout gui/$(id -u) $HOME/Library/LaunchAgents/com.bedichek.watch_gobble_solartracker.plist
ps aux|egrep solartracker
