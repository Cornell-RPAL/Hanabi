def recognize_action(prev_state, new_state, permanence):

        def stateChange(prev_state, new_state): #maybe should change (does only hand matter??)
            visible = self.prev_state
            visible['discard'] = [visible['discard'][0]]
            return visible != new_state

        def updateDiscard(self, new_state):
            self.prev_state = {
            'discard': new_state['discard'] + self.prev_state['discard'],
            'hand': new_state['hand'],
            'board': new_state['board']
            }

        if new_state['gripper']:
            return 'attempt play', new_state['gripper'][0]

        # should be still for 3 frames
        # check if exactly one card id in hand is different

        if stateChange(prev_state, new_state):
            if permanence > 5: #should set in const later

                self.permanence = 0
                if len(new_state['hand']) == 5:

                    new_hand = set(new_state['hand'])
                    old_hand = set(self.prev_state['hand'])
        
                    if len(old_hand - new_hand) == 1:
                        action_card = (old_hand - new_hand).pop()
                        if action_card in new_state['board']:
                            # prev_state = new_state
                            # print('played', action_card)
                            # print('new stable state:', prev_state)
                            return PlayCard(PLAYER, card_ix = new_state['board'].index(action_card))
                        if action_card in new_state['discard']: #could also just check top card
                            # self._updateDiscard(new_state)
                            # print('discarded', action_card)
                            # print('new stable state:', self.prev_state)
                            return Discard(PLAYER, card_ix = new_state['discard'].index(action_card))
                        else:
                            pass
                            # print('a card just disappeared? very bad')
                else:
                    pass
                    # print('waiting to draw new card')
            else:
                pass
                # self._permanence += 1
                # print('waiting for still frame')
        else:
            pass
            # print('nothing is happening')