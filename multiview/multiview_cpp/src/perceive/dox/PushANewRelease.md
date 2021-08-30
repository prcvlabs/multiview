
Pushing a New Release                {#push_a_new_release}
=====================

This is super simple. 

 * Go to the [github multiview page](https://github.com/perceiveinc/multiview)
 * Click on the "releases" hyperlink. (Up and left from the green "clone or download" button.)
 * Click the big "Draft New Release" button. (Up and right of screen.)
 * Give the release a tag like "v3.0.2"
 * Give the release a title like "now using cuda+slic"
 * Write your name in the "describe this release" panel
 * Click the "Publish release" button.
 
## Updating the docker image ##

 * Log onto a `p3` EC2 instance.
 * Clone the latest `multiview` repo.
 * cd to `build` directory.
 * Type: `sudo ./build.sh --verbose --version v3.0.2`. (Obviously you must change the version number.)
 * If this works, then push the new docker container like so: `sudo ./build.sh --publish`

