#  Copyright (C) 2018-2024 LEIDOS.
# 
#  Licensed under the Apache License, Version 2.0 (the "License"); you may not
#  use this file except in compliance with the License. You may obtain a copy of
#  the License at
# 
#  http://www.apache.org/licenses/LICENSE-2.0
# 
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
#  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
#  License for the specific language governing permissions and limitations under
#  the License.

# Change to the directory containing the script
cd "$(dirname "$0")"

# Move up one directory (likely to the root of the project)
cd ..

# Get the last commit message
COMMIT_MSG=$(git log -1 --pretty=%B)

# Remove newlines and special characters, replace spaces with underscores
VERSION=$(echo "$COMMIT_MSG" | tr -d '\n' | tr -dc '[:alnum:][:space:]' | tr '[:space:]' '_')

# Limit the length of the version string (e.g., to 50 characters)
VERSION="${VERSION:0:50}"

# Append -SNAPSHOT if there are uncommitted changes
if [[ -n $(git status -s) ]]; then
    VERSION="${VERSION}-SNAPSHOT"
fi

echo "$VERSION"