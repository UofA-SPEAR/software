Right now, XML linting is accomplished by running each launch file through `xmllint`'s formatting and running a `diff` to compare with the original file. This results in a very strict set of style constraints:

  - Format using 2 spaces for indentation.
  - Use double-quotes `"` instead of single-quotes `'`.
  - No space before each closing tag (e.g. `name="value"/>` instead of `name="value" />`.
  - Blank lines are allowed, but lines containing only whitespace are not. Likewise, lines cannot end with whitespace.
  - Comments are not allowed after a tag. Instead of
    ```xml
    <tag> <!-- comment -->
    ```
    use
    ```xml
    <!-- comment -->
    <tag>
    ```
  - Tags cannot span multiple lines. If a tag is getting too long, consider using `<arg>` tags, e.g. instead of
    ```xml
    <tag attr1="A very very very very very long string" attr2="Another very very very very very long string">
    ```
    use
    ```xml
    <arg name="attr1" value="A very very very very very long string"/>
    <arg name="attr2" value="Another very very very very very long string"/>
    <tag attr1="$(arg attr1)" attr2="$(arg attr2)">
    ```
