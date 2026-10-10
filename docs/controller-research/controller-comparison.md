# Controller research

## Surgeon-Oriented Controller Comparison
> This would likely be a controller to get much later. A lot of specifically surgeon-oriented controllers are **very** expensive.

| Controller | Precision | Joint or Tool-tip | Enable/Deadman | Ergonomics | Haptics | ROS2 | Cost |
| --- | --- | --- | --- | --- | --- | --- | --- |
| [Haply MinVerse](https://www.haply.co/minverse)  | Should have good precision. |  Depending on how it's implemented, it can be either tool-tip or joint controlled | The controller has 2 buttons that can be used for the enable. | Should be very ergonomic. Pencil-shaped. | There can be a force feedback from 2N-4N of resistance and can enable the user to feel textures. | Compatible with ROS2 but might be a bit more effort to integrate.* | $1140+ |
| [SpaceMouse Controller](https://3dconnexion.com/us/product/spacemouse-enterprise/)| Is a 6DOF controller and should be very precise. | Tool-tip controlled | There are many buttons that can be used for an enable switch | This would not be the most ergonomic option for a surgeon since they would control the SpaceMouse with their non-dominant hand. | No haptic feedback | Compatible with the [spacenavd](https://index.ros.org/p/spacenav/) driver package. | $564.41+ | 

> \*There is a [driver](https://www.haply.co/blog/haply-robotics-to-offer-its-haptic-controller-with-pickniks-moveitpro) under the same brand that is compatible with MoveIt, however it's only compatible with the [Inverse3](https://www.haply.co/inverse3) model which is a more advanced controller but is far more expensive. 

## Recommendation

The Haply MinVerse would be a good choice for a long term controller. It is very suitable for controlling surgeries and provides haptics as well. 

Fallback: The SpaceMouse controller would be good for simply controlling the arm. Since it has lots of buttons, many different shortcuts can be mapped and it is a bit cheaper compared to the MinVerse. 
