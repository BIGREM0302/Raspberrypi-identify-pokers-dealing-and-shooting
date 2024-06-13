import random
import time

num_of_player = 4
card_in_hand = 13

class CardType:
    Normal = 0
    Royal_dragon = 1
    Dragon = 2
    Straight_flush = 3
    Four_of_a_kind = 4
    Full_house = 5
    Straight = 6
    Pair = 7
    High_card = 8

def card_num(suit, point):
    return 13 * suit + point

def card_suit(card):
    return card // 13

def card_point(card):
    return card % 13

def shuffle_card(card_deck, used_card, num_of_card):
    available_cards = [i for i, used in enumerate(used_card) if not used]
    random.shuffle(available_cards)
    for i in range(min(num_of_card, len(available_cards))):
        card_deck[i] = available_cards[i]
        used_card[available_cards[i]] = True
    for i in range(num_of_card, 52):
        card_deck[i] = -1

def card_sharp(card_deck, used_card, hand, player, type, wanted_card):
    suit = card_suit(wanted_card)
    point = card_point(wanted_card)
    used_num = [0, 13, 13, 5, 4, 5, 5, 2, 1]
    num_of_card = 52
    
    su = [(1, 2), (1, 3), (2, 3)]
    ty = random.randint(0, 2)
    ano_point = (random.randint(0, 11) + point + 1) % 13
    ano_suit = random.randint(0, 3)
    
    if 0 <= wanted_card <= 51:
        used_card[wanted_card] = True
    
    if type == CardType.Normal:
        pass
    elif type == CardType.Royal_dragon:
        for i in range(13):
            n = card_num(suit, i)
            hand[player][i] = n
            used_card[n] = True
    elif type == CardType.Dragon:
        for i in range(13):
            n = card_num(random.randint(0, 3), i)
            hand[player][i] = n
            used_card[n] = True
    elif type == CardType.Straight_flush:
        for i in range(5):
            n = card_num(suit, (point + 13 - i) % 13)
            hand[player][i] = n
            used_card[n] = True
    elif type == CardType.Four_of_a_kind:
        for i in range(4):
            n = card_num(i, point)
            hand[player][i] = n
            used_card[n] = True
    elif type == CardType.Full_house:
        hand[player][0] = wanted_card
        n = card_num((suit + su[ty][0]) % 4, point)
        hand[player][1] = n
        used_card[n] = True
        n = card_num((suit + su[ty][1]) % 4, point)
        hand[player][2] = n
        used_card[n] = True
        n = card_num(ano_suit, ano_point)
        hand[player][3] = n
        used_card[n] = True
        n = card_num((random.randint(0, 2) + ano_suit + 1) % 4, ano_point)
        hand[player][4] = n
        used_card[n] = True
    elif type == CardType.Straight:
        hand[player][0] = wanted_card
        for i in range(1, 5):
            n = card_num(random.randint(0, 3), (point + 13 - i) % 13)
            hand[player][i] = n
            used_card[n] = True
    elif type == CardType.Pair:
        hand[player][0] = wanted_card
        n = card_num((random.randint(0, 2) + suit + 1) % 4, point)
        hand[player][1] = n
        used_card[n] = True
    elif type == CardType.High_card:
        hand[player][0] = wanted_card
    
    num_of_card -= used_num[type]
    shuffle_card(card_deck, used_card, num_of_card)
    
    count = 0
    for i in range(used_num[type], card_in_hand):
        hand[player][i] = card_deck[count]
        count += 1
    for i in range(num_of_player):
        if i != player:
            for j in range(card_in_hand):
                hand[i][j] = card_deck[count]
                count += 1

def generation():
    random.seed(int(time.time()))
    card_deck = [0] * 52
    used_card = [False] * 52
    hand = [[0] * card_in_hand for _ in range(num_of_player)]
    final_deck = [0] * 52

    player = 0
    print("Normal = 0 \r\nRoyal_dragon = 1 \r\nDragon = 2\r\nStraight_flush = 3\r\nFour_of_a_kind = 4\r\nFull_house = 5\r\nStraight = 6\r\nPair = 7\nHigh_card = 8")
    type = int(input("please input the deal type: "))
    wanted_card = card_num(3, 0)

    card_sharp(card_deck, used_card, hand, player, type, wanted_card)

    for i in range(num_of_player):
        for j in range(card_in_hand):
            final_deck[hand[i][j]] = i + 1
    return final_deck

if __name__ == "__main__":
    matrix = generation()
    print(matrix)
