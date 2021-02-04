#!/bin/bash

# List of project folders.
declare -r ESLINTRC=".eslintrc.json"
declare -ar PROJECTS=("../mission-planner-gui" "../rover-map-view")

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
        npm install eslint-plugin-react@latest eslint-config-google@latest eslint
    fi
done

# Runs ESLint on all of the JavaScript projects.
for PROJECT in ${PROJECTS[@]}
do
    echo "Linting ${PROJECT}"
    npx eslint src/*.js
done

