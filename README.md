This is a ROS wrapper for the Azure Spatial Anchors Linux SDK, allowing robots (and other devices equipped with a vision-based sensors and a pose estimation system) to create and query [Azure Spatial Anchors](https://azure.microsoft.com/en-us/services/spatial-anchors/), allowing the robot to co-localize with AR-enabled phones and Hololens devices.

This is presented as part of the [IROS 2020 Tutorial on Mixed Reality and Robotics](https://www.microsoft.com/en-us/research/event/mixed-reality-and-robotics-tutorial-iros-2020/). Tutorial videos are available for free starting October 25th through [IROS 2020](https://www.iros2020.org/).

You can sign up to get access to the Azure Spatial Anchors Linux SDK here: [aka.ms/ASALinuxSDKSignup](http://aka.ms/ASALinuxSDKSignup).
**This is REQUIRED to use this repo.** It may take us up to 2 business days to add you. This requires an [Azure Devops account](https://azure.microsoft.com/en-us/services/devops/).

You should have an Azure Spatial Anchors account in Azure to use this SDK and wrapper.
Please see instructions for the [ASA Quickstarts](https://docs.microsoft.com/en-us/azure/spatial-anchors/quickstarts/get-started-unity-hololens) to create one. Keep the Account ID, and Account Key, and Account Domain nearby. This is free up to 10k queries/month.

# Requirements
- Ubuntu 18.04/20.04
- ROS melodic or noetic
- [Azure Portal](https://azure.microsoft.com/account/portal) account with an Azure Spatial Anchors resource.
- [Azure Devops account](https://azure.microsoft.com/en-us/services/devops/) account.
- [Signed up for Azure Spatial Anchors Linux SDK](http://aka.ms/ASALinuxSDKSignup), which may take 1-2 days.

# Input data requirements
Timeseries of:
- *Undistorted* grayscale images
- 6 DoF poses of the cameras for the corresponding images
- Camera intrinsics (focal length, principal point)

See this description of [camera calibration](https://ch.mathworks.com/help/vision/ug/camera-calibration.html) for more details.

# Get Started
See the **[Wiki](https://github.com/microsoft/azure_spatial_anchors_ros/wiki) for getting started and installation instructions**.


# Support
Check out the [FAQ and Known Issues](https://github.com/microsoft/azure_spatial_anchors_ros/wiki/FAQ-&-Known-Issues) page on the Wiki first!

Please file an [issue on the Github repo](https://github.com/microsoft/azure_spatial_anchors_ros/issues) if it is not covered in the Wiki.

# Contributing

This project welcomes contributions and suggestions.  Most contributions require you to agree to a
Contributor License Agreement (CLA) declaring that you have the right to, and actually do, grant us
the rights to use your contribution. For details, visit https://cla.opensource.microsoft.com.

When you submit a pull request, a CLA bot will automatically determine whether you need to provide
a CLA and decorate the PR appropriately (e.g., status check, comment). Simply follow the instructions
provided by the bot. You will only need to do this once across all repos using our CLA.

This project has adopted the [Microsoft Open Source Code of Conduct](https://opensource.microsoft.com/codeofconduct/).
For more information see the [Code of Conduct FAQ](https://opensource.microsoft.com/codeofconduct/faq/) or
contact [opencode@microsoft.com](mailto:opencode@microsoft.com) with any additional questions or comments.
