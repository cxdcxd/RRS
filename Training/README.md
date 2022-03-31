## Setup Server Build 

1. In unity editor Select File -> Build Settings.
2. Choose Target Platform `Windows` and Architecture `x86_64`.
3. Click Build and choose appropriate folder to save Server Build.

## ML Agents Learning

Use Server Build for ML Agents Training by following:

`mlagents-learn TrainingConfig\ppo.yaml.txt --env="<Path To Server Build Executable>" --run-id="<Some Unique Name>"`
