"""
데이터베이스 연습 #318
SQLite CRUD 실습
"""
import sqlite3

def setup_318():
    conn = sqlite3.connect(':memory:')
    cursor = conn.cursor()
    cursor.execute('''CREATE TABLE items_318 (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL,
        value REAL,
        category TEXT
    )''')

    data = [
        [('item_318_0', 248.56, 'A'), ('item_318_1', 307.73, 'C'), ('item_318_2', 152.91, 'B'), ('item_318_3', 915.59, 'C'), ('item_318_4', 418.55, 'B'), ('item_318_5', 561.71, 'A'), ('item_318_6', 394.53, 'B'), ('item_318_7', 211.3, 'C')]
    ]
    cursor.executemany(
        'INSERT INTO items_318 (name, value, category) VALUES (?, ?, ?)',
        data
    )
    conn.commit()
    return conn

if __name__ == '__main__':
    conn = setup_318()
    cursor = conn.cursor()

    print(f"DB 연습 #318")
    cursor.execute('SELECT * FROM items_318')
    for row in cursor.fetchall():
        print(f"  {row}")

    cursor.execute('SELECT category, COUNT(*), AVG(value) FROM items_318 GROUP BY category')
    print("\n카테고리별 통계:")
    for row in cursor.fetchall():
        print(f"  {row[0]}: {row[1]}개, 평균 {row[2]:.2f}")

    conn.close()
