# Publish package to PyPi

Firstly you have to ensure, that package properly builds, publishes and works. 
To do this, you must first publish it on https://test.pypi.org

1. Create tag on main branch:
    ```commandline
    git tag <tagname>
    ```
2. Push created tag to repository
    ```commandline
    git push origin <tagname>
    ```
   After this GitHubActions will automatically build pip package and push it to test PyPi registry
3. Ensure, that installed pip package from https://test.pypi.org works properly
   ```commandline
    pip install --index-url https://test.pypi.org/simple/ --extra-index-url https://pypi.org/simple sova==<tagname>
    ```
4. Publish pip package manually on production PyPi using the following steps:
   - Go to Actions page
   - Go to "Publish pip on PyPi" workflow on left side of the screen
   - Go to "Run workflow" and choose last tag which has been created from tags
   - Click "Run workflow"

   After this GitHubActions will automatically build pip package and push it to test production registry
