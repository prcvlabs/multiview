
# Multiview Documentation

```bash
    aws-bin/      # Directory of command-line scripts that execute on EC2 instances.
    bin/          # Directory of command-line scripts.
    doc/          # Documentation directory.
        build.sh  # Command to build documentation.
        site/     # Built documentation.
        src/      # Documentation source.
    gui/          # Directory of guis.
    install.sh    # Installation script. Run once.
    multiview/    # Source for the multiview package.
    README.md     # Github readme file.
    samples/      # Code samples double as simple regression tests.
    scraps/       # Testing and debug commands -- may be useful as reference.
```

## Install

```bash
 > sudo ./install.sh
```

### PERCEIVE_DATA and MULTIVIEW_DIR

PERCEIVE_DATA should point at the perceive-data dropbox directory on your machine.
Production code must not use the PERCEIVE_DATA directory, but it is can be used
on ec2 instances. (See the examples in the `job-runner` repo.)

MULTIVIEW_DIR should point to the multiview directory -- the one with the project
directory below. This environment variable is mainly useful for the `job-runner`.

```bash
 > echo "export PERCEIVE_DATA=/path/to/your/perceive-data/directory" >> ~/.bashrc
 > echo "export MULTIVIEW_DIR=/path/to/your/perceive/multiview" >> ~/.bashrc
```

## Howto

Check out the howto pages to see if there's some code that is similar
to what you need.

