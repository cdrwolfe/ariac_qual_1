#!/usr/bin/env python

# Orders module, handles information regarding all orders
class Orders:
    # An orders class
    def __init__(self, order):
        # Initialise
        self.ID = order.order_id
        self.kits = []
        kit = {}
        for x in order.kits:
            # Add kit
            kit['Type'] = x.kit_type
            kit['Objects'] = x.objects
            self.kits.append(kit)
