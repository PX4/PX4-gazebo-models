#!/bin/bash

PREV=$1
CURR=$2
FUEL_TOKEN=$3

mod_list=$(git diff $PREV $CURR --name-status)
prev_model=


while IFS= read -r line; do

	type=$(echo "$line" | awk -F'/' '{print $1}' | awk -F' ' '{print $2}')
	model=$(echo "$line" | awk -F'/' '{print $2}')

	# Delete the model from if it exists and is encountered for the first time.
	if [[ $model != $prev_model ]] && ([[ $type == "models" ]] || [[ $type == "worlds" ]]); then
		echo "Trying to delete $type/$model from fuel server"
		gz fuel delete --header "Private-token: $FUEL_TOKEN" --url https://fuel.gazebosim.org/1.0/PX4/$type/$model -o PX4

	# If you want to modify or add a model, reupload it.
		if [[ "$line" == "A"* || "$line" == "M"* ]]; then
				echo "Uploading $type/$model to fuel server"
				gz fuel upload -m ./$type/$model -o PX4 --header "Private-token: $FUEL_TOKEN" --url https://fuel.gazebosim.org
		fi

		prev_model=$model

	else
		# If the model is just to be deleted, continue.
		continue
	fi


done <<< "$mod_list"
