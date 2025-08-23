"""
데이터베이스 연습 #342
SQLite CRUD 실습
"""
import sqlite3

def setup_342():
    conn = sqlite3.connect(':memory:')
    cursor = conn.cursor()
    cursor.execute('''CREATE TABLE items_342 (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL,
        value REAL,
        category TEXT
    )''')

    data = [
        [('item_342_0', 372.14, 'A'), ('item_342_1', 462.97, 'B'), ('item_342_2', 673.99, 'B'), ('item_342_3', 532.44, 'C'), ('item_342_4', 320.27, 'B'), ('item_342_5', 615.2, 'C'), ('item_342_6', 123.09, 'B')]
    ]
    cursor.executemany(
        'INSERT INTO items_342 (name, value, category) VALUES (?, ?, ?)',
        data
    )
    conn.commit()
    return conn

if __name__ == '__main__':
    conn = setup_342()
    cursor = conn.cursor()

    print(f"DB 연습 #342")
    cursor.execute('SELECT * FROM items_342')
    for row in cursor.fetchall():
        print(f"  {row}")

    cursor.execute('SELECT category, COUNT(*), AVG(value) FROM items_342 GROUP BY category')
    print("\n카테고리별 통계:")
    for row in cursor.fetchall():
        print(f"  {row[0]}: {row[1]}개, 평균 {row[2]:.2f}")

    conn.close()
