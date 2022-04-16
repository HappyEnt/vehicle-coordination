# Components
This directory contains all the relevant components developed in this project. Each should act as an ordinary component, not knowing it has been moved to a project with other components.
Please try to separate the components as if they have been developed in their own repositories, i.e. with own README, setup etc.
This overall project repository can then focus on the integration of the different components and point to the individual component documentations.

You can easily add and pull commits from other repositories into the components' directory using `git-subtree` (see [this stackoverflow post](https://stackoverflow.com/a/30386041/6669161) ("4. Whole repo git subtree"):
Adding a component:
```
git subtree add --prefix=components/<component> https://github.com/user/<component>.git master
```

Pulling component changes:
```
git subtree pull --prefix=components/<component> https://github.com/user/<component>.git master
```
(You could also squash your commits using `--squash`)


