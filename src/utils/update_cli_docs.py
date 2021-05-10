#!/usr/bin/env python3

import optparse
import os
import re
import yaml  # pyyaml / python-yaml

SETTINGS_MD_PATH = "docs/Settings.md"
SETTINGS_YAML_PATH = "src/main/fc/settings.yaml"
CODE_DEFAULTS_PATH = "src/main"

DEFAULTS_BLACKLIST = [
    'baro_hardware',
    'dterm_lpf_type',
    'dterm_lpf2_type',
    'failsafe_procedure',
    'flaperon_throw_offset',
    'heading_hold_rate_limit',
    'mag_hardware',
    'pitot_hardware',
    'rx_min_usec',
    'serialrx_provider',
]

MIN_MAX_REPLACEMENTS = {
    'INT16_MIN': -32768,
    'INT16_MAX': 32767,
    'INT32_MIN': -2147483648,
    'INT32_MAX': 2147483647,
    'UINT8_MAX': 255,
    'UINT16_MAX': 65535,
    'UINT32_MAX': 4294967295,
}

def parse_settings_yaml():
    """Parse the YAML settings specs"""

    with open(SETTINGS_YAML_PATH, "r") as settings_yaml:
        return yaml.load(settings_yaml, Loader=yaml.Loader)

def generate_md_table_from_yaml(settings_yaml):
    """Generate a sorted markdown table with description & default value for each setting"""
    params = {}
    
    # Extract description, default/min/max values of each setting from the YAML specs (if present)
    for group in settings_yaml['groups']:
        for member in group['members']:
            if not any(key in member for key in ["description", "default_value", "min", "max"]) and not options.quiet:
                print("Setting \"{}\" has an incomplete specification".format(member['name']))

            # Handle default/min/max fields for each setting
            for key in ["default_value", "min", "max"]:
                # Basing on the check above, not all fields may be present
                if key in member:
                    ### Fetch actual values from the `constants` block if present
                    if ('constants' in settings_yaml) and (member[key] in settings_yaml['constants']):
                        member[key] = settings_yaml['constants'][member[key]]
                    ### Fetch actual values from hardcoded min/max replacements
                    elif member[key] in MIN_MAX_REPLACEMENTS:
                        member[key] = MIN_MAX_REPLACEMENTS[member[key]]

                    ### Handle edge cases of YAML autogeneration and prettify some values
                    # Replace booleans with "ON"/"OFF"
                    if type(member[key]) == bool:
                        member[key] = "ON" if member[key] else "OFF"
                    # Replace zero placeholder with actual zero
                    elif member[key] == ":zero":
                        member[key] = 0
                    # Replace target-default placeholder with extended definition
                    elif member[key] == ":target":
                        member[key] = "_target default_"
                    # Replace empty strings with more evident marker
                    elif member[key] == "":
                        member[key] = "_empty_"
                    # Reformat direct code references
                    elif str(member[key])[0] == ":":
                        member[key] = f'`{member[key][1:]}`'


            params[member['name']] = {
                    "description": member["description"] if "description" in member else "",
                    "default": member["default_value"] if "default_value" in member else "",
                    "min": member["min"] if "min" in member else "",
                    "max": member["max"] if "max" in member else ""
                }
    
    # MD table header
    md_table_lines = [
        "| Variable Name | Default Value | Min | Max | Description |\n",
        "| ------------- | ------------- | --- | --- | ----------- |\n",
        ]
    
    # Sort the settings by name and build the rows of the table
    for param in sorted(params.items()):
        md_table_lines.append("| {} | {} | {} | {} | {} |\n".format(
            param[0], param[1]['default'], param[1]['min'], param[1]['max'], param[1]['description']
        ))
    
    # Return the assembled table
    return md_table_lines

def write_settings_md(lines):
    """Write the contents of the CLI settings docs"""

    with open(SETTINGS_MD_PATH, "w") as settings_md:
        settings_md.writelines(lines)

# Return all matches of a compiled regex in a list of files
def regex_search(regex, files):
    for f in files:
        with open(f, 'r') as _f:
            for _, line in enumerate(_f.readlines()):
                matches = regex.search(line)
                if matches:
                    yield matches

# Return plausible default values for a given setting found by scraping the relative source files
def find_default(setting_name, headers):
    regex = re.compile(rf'^\s*\.{setting_name}\s=\s([A-Za-z0-9_\-]+)(?:,)?(?:\s+//.+$)?')
    files_to_search_in = []
    for header in headers:
        header = f'{CODE_DEFAULTS_PATH}/{header}'
        files_to_search_in.append(header)
        if header.endswith('.h'):
            header_c = re.sub(r'\.h$', '.c', header)
            if os.path.isfile(header_c):
                files_to_search_in.append(header_c)
    defaults = []
    for matches in regex_search(regex, files_to_search_in):
        defaults.append(matches.group(1))
    return defaults

# Try to map default values in the YAML spec back to the actual C code and check for mismatches (defaults updated in the code but not in the YAML)
# Done by scraping the source files, prone to false negatives. Settings in `DEFAULTS_BLACKLIST` are ignored for this
# reason (values that refer to other source files are too complex to parse this way).
def check_defaults(settings_yaml):
    retval = True
    for group in settings_yaml['groups']:
        if 'headers' in group:
            headers = group['headers']
            for member in group['members']:
                # Ignore blacklisted settings
                if member['name'] in DEFAULTS_BLACKLIST:
                    continue

                default_from_code = find_default(member['name'], headers)
                if len(default_from_code) == 0: # No default value found (no direct code mapping)
                    continue
                elif len(default_from_code) > 1: # Duplicate default values found (regexes are a quick but poor solution)
                    if not options.quiet:
                        print(f"Duplicate default values found for {member['name']}: {default_from_code}, consider adding to blacklist")
                    continue

                # Extract the only matched value, guarded by the previous checks
                default_from_code = default_from_code[0]
                # Map C values to their equivalents used in the YAML spec
                code_values_map = { 'true': 'ON', 'false': 'OFF' }
                if default_from_code in code_values_map:
                    default_from_code = code_values_map[default_from_code]
                
                default_from_yaml = member["default_value"] if "default_value" in member else ""
                # Remove eventual Markdown formatting
                default_from_yaml = default_from_yaml.replace('`', '').replace('*', '').replace('__', '')
                # Allow specific C-YAML matches that coudln't be replaced in the previous steps
                extra_allowed_matches = { '1': 'ON', '0': 'OFF' }
                
                if default_from_yaml not in default_from_code: # Equal or substring
                    if default_from_code in extra_allowed_matches and default_from_yaml in extra_allowed_matches[default_from_code]:
                        continue
                    if not options.quiet:
                        print(f"{member['name']} has mismatched default values. Code reports '{default_from_code}' and YAML reports '{default_from_yaml}'")
                    retval = False
    return retval

if __name__ == "__main__":
    global options, args
    parser = optparse.OptionParser()
    parser.add_option('-q', '--quiet', action="store_true", default=False, help="do not write anything to stdout")
    parser.add_option('-d', '--defaults', action="store_true", default=False, help="check for mismatched default values")
    options, args = parser.parse_args()

    settings_yaml = parse_settings_yaml()

    if options.defaults:
        defaults_match = check_defaults(settings_yaml)
        quit(0 if defaults_match else 1)
    
    md_table_lines = generate_md_table_from_yaml(settings_yaml)
    settings_md_lines = \
        ["# CLI Variable Reference\n", "\n" ] + \
        md_table_lines + \
        ["\n", "> Note: this table is autogenerated. Do not edit it manually."]
    write_settings_md(settings_md_lines)
