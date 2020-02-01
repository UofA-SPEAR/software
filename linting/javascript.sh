#!/bin/bash

# List of project folders.
declare -r ESLINTRC=".eslintrc.json"
declare -ar PROJECTS=("../mission-planner-gui" "../rover-map-view")

# Installs ESLint if not already installed.
if ! [ -x "$(command -v eslint)" ]
then
    npm install -g eslint
fi

# Prepares all of the JavaScript projects.
for PROJECT in ${PROJECTS[@]}
do
    echo "Setting up ${PROJECT}"
    cp ${ESLINTRC} ${PROJECT}/.eslintrc.json
    cd ${PROJECT}
    # Installs the project dependencies if not already available.
    if ! [ -d "node_modules" ]
    then
        npm install
        npm install eslint-plugin-react@latest eslint-config-google@latest
    fi
done

# Runs ESLint on all of the JavaScript projects.
for PROJECT in ${PROJECTS[@]}
do
    echo "Linting ${PROJECT}"
    eslint src/*.js
done

