#!/bin/bash

# This script converts text files into one HTML file.

# List of files to convert (glob)
FILES=(
  "./README.md"
  "./src/*"
  "./src/**/*"
  "./scripts/*"
  "./include/gif-pros/*"
  "./include/robot/*"
)

# Ext override map
EXT_OVERRIDES=(
  "hpp:cpp"
  "sh:bash"
  "h:c"
)

# Output file
OUTPUT="index.html"

# Header
cat <<EOF > $OUTPUT
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <title>My Project</title>

  <!-- PrismJS -->
  <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/prism/9000.0.1/themes/prism-coy.min.css" />
  <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/prism/9000.0.1/plugins/line-numbers/prism-line-numbers.min.css" />

  <style>
    body {
      font-family: sans-serif;
      margin: 0;
      padding: 8px;
    }
    pre {
      margin: 0;
    }

    code {
      font-family: monospace;
    }

    h1 {
      margin-top: 1rem;
      margin-bottom: 0.25rem;
    }

    p {
      margin-top: 0.25rem;
      margin-bottom: 0.25rem;
    }

    pre[class*="language-"] {
      max-height: inherit !important;
      overflow: hidden !important;

      box-shadow: none !important;
      border-left: none !important;
    }

    code[class*="language"] {
      overflow: hidden !important;
    }
  </style>
</head>
<body>
EOF

# Append each file to the output with a header of file path
for FILE in ${FILES[@]}; do
  # Ignore directories
  if [ -d "$FILE" ]; then
    continue
  fi

  FILE_EXT="${FILE##*.}"

  # if markdown, convert to html
  if [ "$FILE_EXT" = "md" ]; then
    FILE_EXT="html"
    FILE="$FILE"
    pandoc -f markdown -t html $FILE >> $OUTPUT
    echo "<hr>" >> $OUTPUT

    continue
  fi

  # Check if the file extension is overridden
  for OVERRIDE in ${EXT_OVERRIDES[@]}; do
    EXT=${OVERRIDE%:*}
    NEW_EXT=${OVERRIDE#*:}
    if [ "$FILE_EXT" = "$EXT" ]; then
      FILE_EXT="$NEW_EXT"
    fi
  done

  # Append the title
  echo "<h1>$FILE</h1>" >> $OUTPUT
  
  # Get the last git commit message and date
  LAST_COMMIT=$(git log -1 --pretty=format:"%s (%ad)" --date=short $FILE)
  echo "<p><strong>Last commit:</strong> $LAST_COMMIT</p>" >> $OUTPUT

  # Append the code
  echo "<pre><code class=\"language-$FILE_EXT line-numbers\">" >> $OUTPUT

  cat $FILE | sed 's/&/\&amp;/g; s/</\&lt;/g; s/>/\&gt;/g; s/"/\&quot;/g; s/'"'"'/\&#39;/g' >> $OUTPUT
  echo "</code></pre>" >> $OUTPUT

  # cat $FILE | sed -e 's/^/    /' | sed -e 's/  / \&nbsp;/' | sed -e 's/$/<br>/' >> $OUTPUT
done

# Footer
cat <<EOF >> $OUTPUT
  <!-- PrismJS -->
  <script src="https://cdnjs.cloudflare.com/ajax/libs/prism/9000.0.1/prism.min.js"></script> 
  <script src="https://cdnjs.cloudflare.com/ajax/libs/prism/9000.0.1/components/prism-c.min.js"></script>
  <script src="https://cdnjs.cloudflare.com/ajax/libs/prism/9000.0.1/components/prism-cpp.min.js"></script>
  <script src="https://cdnjs.cloudflare.com/ajax/libs/prism/9000.0.1/components/prism-bash.min.js"></script>
  <script src="https://cdnjs.cloudflare.com/ajax/libs/prism/9000.0.1/plugins/line-numbers/prism-line-numbers.min.js"></script>
</body>
</html>
EOF

