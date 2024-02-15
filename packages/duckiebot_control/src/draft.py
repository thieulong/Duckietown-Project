# Original dictionary
tag_id_dict = {
    "Stop": [31, 32, 33],
    "T-Intersection": 61,
    "Side Road Left": 65,
    "Side Road Right": 57
}

# Create a reverse mapping
reverse_dict = {val: key for key, values in tag_id_dict.items() if isinstance(values, list) for val in values}
reverse_dict.update({val: key for key, val in tag_id_dict.items() if not isinstance(val, list)})

print(reverse_dict)

# Test
id_to_find = 32
if id_to_find in reverse_dict:
    relevant_value = reverse_dict[id_to_find]
    print(f"For id {id_to_find}, the relevant value is: {relevant_value}")
else:
    print(f"No matching value found for id {id_to_find}")
