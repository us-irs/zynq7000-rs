target remote localhost:3000

# *try* to stop at the user entry point (it might be gone due to inlining)
break main

load

continue
