#include <class_loader/class_loader.hpp>
#include "example_navigate/temoto_action.hpp"

#include "find_coordinates.cpp"


#include <fmt/core.h>
#include <chrono>
#include <thread>

class FindCoordinates : public TemotoAction
{
public:

FindCoordinates() // REQUIRED
{
}

void onInit()
{
  TEMOTO_PRINT_OF("Initializing", getName());
}

bool onRun() // REQUIRED
{






  TEMOTO_PRINT_OF("Done\n", getName());

  return true;
}

void onPause()
{
  TEMOTO_PRINT_OF("Pausing", getName());
}

void onContinue()
{
  TEMOTO_PRINT_OF("Continuing", getName());
}

void onStop()
{
  TEMOTO_PRINT_OF("Stopping", getName());
}

~FindCoordinates()
{
}

}; // FindCoordinates class

/* REQUIRED BY CLASS LOADER */
CLASS_LOADER_REGISTER_CLASS(FindCoordinates, ActionBase);
