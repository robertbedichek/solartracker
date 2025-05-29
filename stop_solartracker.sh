#!/bin/zsh

sudo launchctl bootout gui/$(id -u) /Users/robertbedichek/Library/LaunchAgents/com.bedichek.watch_gobble_solartracker.plist
ps aux|egrep gobble_solartracker
